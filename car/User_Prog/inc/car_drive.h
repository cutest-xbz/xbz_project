/*******************************************************************************
 * @file    stm32f103_car_drive.h
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   小车控制模块 —— 应用组合层
 *
 * 将 Motor（电机）、Tracker（传感器）、PID（算法）组合为一个完整的
 * 智能小车差速转向控制系统。遵循分层架构中的"上层只调紧邻下层"铁律。
 *
 * ── 控制策略：开环差速转向 ──
 *
 *              循迹误差 [-1, +1]
 *                   │
 *                   ▼
 *   ┌─────────────────────────────────┐
 *   │         转向 PID（增量式）       │
 *   │   目标 = 0（居中）               │
 *   │   反馈 = 当前循迹误差            │
 *   │   输出 = 左右轮 PWM 修正量       │
 *   └──────────────┬──────────────────┘
 *                   │  turn_output
 *         ┌─────────┴─────────┐
 *         ▼                   ▼
 *   left  = base + turn   right = base - turn
 *   (右偏→左轮加速→左转)   (右偏→右轮减速→左转)
 *         │                   │
 *         ▼                   ▼
 *   MOTOR_setSpeed(L)    MOTOR_setSpeed(R)
 *
 * ── 为什么用增量式 PID？ ──
 *   1. 天然抗积分饱和：输出增量有界，即使误差持续存在也不会发散
 *   2. 输出平滑：相邻周期输出变化小，不会突变导致小车抖动
 *   3. 适合 MCU：计算量小（3 次乘法 + 3 次加法），10ms 周期绰绰有余
 *
 * ── 依赖注入 ──
 *   CARDRIVE 不直接创建 Motor/Tracker 实例，而是通过 Init 前设置
 *   CARx->hd.motor_L / motor_R / tracker 指针接收已初始化的下层句柄。
 *   这实现了控制反转（IoC）：上层定义控制逻辑，下层提供硬件能力。
 *
 * ── 使用示例 ──
 *   // main.c 初始化序列
 *   CAR1->hd.motor_L = MOTOR1;
 *   CAR1->hd.motor_R = MOTOR2;
 *   CAR1->hd.tracker = TRACKER1;
 *   CARDRIVE_InitTypeDef_Struct cfg;
 *   CARDRIVE_StructInit(&cfg);
 *   CARDRIVE_Init(CAR1, &cfg);
 *   // 每 10ms 控制循环
 *   CARDRIVE_run(CAR1);
 *******************************************************************************/

#ifndef __CAR_DRIVE_H
#define __CAR_DRIVE_H

#include "stm32f10x.h"
#include "motor.h"
#include "tracker.h"
#include "pid.h"

/*===========================================================================
 * 1. 默认值宏定义
 *===========================================================================*/
#define CARDRIVE_ENABLE_DEFAULT      ENABLE    /* 上电默认使能 */
#define CARDRIVE_BASEPWM_DEFAULT     (0.5f)    /* 基础速度 40% PWM（约 0.4m/s） */
#define CARDRIVE_SPEEDMAX_DEFAULT    (1.0f)    /* 最大输出 100% PWM */

/* 转向 PID 默认参数（推荐调参顺序：先 Kp → 再 Kd → 最后 Ki） */
#define CARDRIVE_KP_DEFAULT          (1.2f)    /* 比例：归中力度，U型弯起步值 */
#define CARDRIVE_KI_DEFAULT          (0.0f)    /* 积分：循迹无稳态误差需求，保持 0 */
#define CARDRIVE_KD_DEFAULT          (0.4f)    /* 微分：抑制过冲，弯道阻尼 */
#define CARDRIVE_PID_OUTMAX_DEFAULT  (0.8f)    /* PID 输出限幅 ±50%（宽弯无需极限差速） */
#define CARDRIVE_CURVE_SLOWDOWN      (0.0f)    /* 弯道减速系数 [0~1]，0=不减速 */

/*===========================================================================
 * 2. 结构体定义
 *===========================================================================*/

/**
 * @brief 小车硬件描述 —— 注入的下层句柄
 *
 * 三个指针在 Init 前由 main.c 设置：
 *   CAR1->hd.motor_L = MOTOR1;  // 左电机（已注入 PWM1）
 *   CAR1->hd.motor_R = MOTOR2;  // 右电机（已注入 PWM2）
 *   CAR1->hd.tracker = TRACKER1; // 灰度传感器
 */
typedef struct {
    MOTOR_TypeDef_Struct    *motor_L;
    MOTOR_TypeDef_Struct    *motor_R;
    TRACKER_TypeDef_Struct  *tracker;
} CARDRIVE_hd_Struct;

/**
 * @brief 小车运行参数
 *
 * basePWM: 直行基础速度。调参时先定此值让小车稳定直行，再调 PID。
 * speedMax: 输出限幅。防止差速过大导致一侧车轮停止或反转。
 */
typedef struct {
    FunctionalState  enable;
    float            basePWM;
    float            speedMax;
} CARDRIVE_mem_Struct;

/**
 * @brief 小车合并句柄
 *
 * pid_turn 嵌入在句柄中而非单独注入，因为 PID 是纯算法模块，
 * 不涉及硬件资源，作为内部组件更简洁。
 */
typedef struct {
    CARDRIVE_hd_Struct   hd;
    CARDRIVE_mem_Struct  mem;
    PID_t                pid_turn;
} CARDRIVE_TypeDef_Struct;

/**
 * @brief 小车初始化配置结构体
 */
typedef struct {
    FunctionalState  enable;
    float            basePWM;
    float            speedMax;
} CARDRIVE_InitTypeDef_Struct;

/*===========================================================================
 * 3. 全局实例池 + 宏别名
 *===========================================================================*/
#define CARDRIVE_BASE_NUM   1
extern CARDRIVE_TypeDef_Struct CARDRIVE_BASE[CARDRIVE_BASE_NUM];

#define CAR1               (&CARDRIVE_BASE[0])

/*===========================================================================
 * 4. API 函数声明
 *
 * CARDRIVE_run 是整个系统唯一需要在控制周期中调用的函数。
 * 它内部完成传感器读取→PID计算→电机输出的完整链路。
 *===========================================================================*/
ErrorStatus CARDRIVE_StructInit(CARDRIVE_InitTypeDef_Struct *p);
ErrorStatus CARDRIVE_Init(CARDRIVE_TypeDef_Struct *CARx, CARDRIVE_InitTypeDef_Struct *CARDRIVE_InitStruct);
ErrorStatus CARDRIVE_Enable(CARDRIVE_TypeDef_Struct *CARx);
ErrorStatus CARDRIVE_Disable(CARDRIVE_TypeDef_Struct *CARx);
ErrorStatus CARDRIVE_run(CARDRIVE_TypeDef_Struct *CARx);
ErrorStatus CARDRIVE_EmergencyStop(CARDRIVE_TypeDef_Struct *CARx);
ErrorStatus CARDRIVE_setBasePWM(CARDRIVE_TypeDef_Struct *CARx, float pwm);
ErrorStatus CARDRIVE_tunePID(CARDRIVE_TypeDef_Struct *CARx, float kp, float ki, float kd);

#endif /* __CAR_DRIVE_H */
