/*******************************************************************************
 * @file    stm32f103_motor.h
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   电机模块 —— 应用层
 *
 * 封装 TB6612 电机驱动板的完整控制逻辑，包括：
 *   1. 方向控制：通过 4 个 GPIO 实现正转/反转/刹车/惰行
 *   2. 速度控制：通过注入的 PWM 句柄调用下层 PWM_setDuty()
 *   3. 编码器读取：AB 相增量式编码器，16→32 位扩展，低通滤波
 *
 * 依赖注入：
 *   本模块不直接访问 PWM 全局变量，而是在 Init 前由上层设置
 *   MOTORx->hd.PWMx = PWM1/PWM2，运行时通过句柄调用 PWM API。
 *   这遵循"上层只调用紧邻下层"的分层铁律。
 *
 * signed speed 设计：
 *   MOTOR_setSpeed 使用带符号的速度值：
 *     speed > 0  → 正转（前进方向）
 *     speed < 0  → 反转（后退方向）
 *     speed = 0  → 短接刹车（快速停止）
 *   相较于传统 dir+duty 分离接口，signed speed 更直观，
 *   且天然适配 PID 输出的有符号修正量。
 *
 * 硬件资源（每个电机）：
 *   MOTOR1(左): PWM1(TIM3_CH1/PA6), PD9(IN1), PD11(IN2), TIM2编码器(PA0/PA1)
 *   MOTOR2(右): PWM2(TIM3_CH2/PA7), PD12(IN1), PD10(IN2), TIM4编码器(PB6/PB7)
 *
 * 使用示例：
 *   // 依赖注入
 *   MOTOR1->hd.PWMx = PWM1;
 *   MOTOR_InitTypeDef_Struct cfg;
 *   MOTOR_StructInit(&cfg);
 *   MOTOR_Init(MOTOR1, &cfg);
 *   // 运行时
 *   MOTOR_setSpeed(MOTOR1, 0.5f);   // 左电机 50% 正转
 *   MOTOR_setSpeed(MOTOR1, -0.3f);  // 左电机 30% 反转
 *   MOTOR_setSpeed(MOTOR1, 0.0f);   // 左电机刹车
 *******************************************************************************/

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "pwm.h"

/*===========================================================================
 * 1. 默认值宏定义
 *===========================================================================*/
#define MOTOR_ENABLE_DEFAULT     ENABLE        /* 上电默认使能 */
#define MOTOR_SPEED_DEFAULT      (0.0f)        /* 初始速度 0（停止） */
#define MOTOR_SPEED_MAX          (1.0f)        /* 正转最大速度（对应 100% PWM） */
#define MOTOR_SPEED_MIN          (-1.0f)       /* 反转最大速度（对应 -100% PWM） */
#define MOTOR_DUTY_MAX           (1.0f)        /* PWM 占空比上限 */
#define MOTOR_DUTY_MIN           (0.0f)        /* PWM 占空比下限 */

/*
 * 编码器低通滤波系数 α（0=无滤波仅新值, 1=完全旧值无响应）
 * 公式：filtered = α×old + (1-α)×new
 * α=0.3 表示新值权重 70%，在平滑度和响应速度间平衡。
 * 增大 → 更平滑但响应慢；减小 → 响应快但噪声大。
 */
#define MOTOR_ENC_FILTER         0.3f

/*
 * 编码器每圈脉冲数（4 倍频前）
 * 390 线编码器 ×4 倍频 = 1560 脉冲/圈。
 * 配合 65mm 轮径：每脉冲 ≈ 0.13mm 线位移。
 */
#define MOTOR_ENC_PPR            390

/*===========================================================================
 * 2. 结构体定义
 *
 * 注意：与 PWM/Tracker 不同，Motor 不是最底层硬件模块。
 * 它的 hd 中包含指向 PWM 的指针（依赖注入），而非直接包含定时器。
 * 方向引脚和编码器定时器仍是硬件绑定（编译时确定）。
 *===========================================================================*/

/**
 * @brief 电机硬件描述结构体
 *
 * PWMx: 注入的 PWM 句柄指针，由上层在 Init 前设置。
 *       电机不负责初始化 PWM，只负责在运行时调用 PWM_setDuty()。
 * CtrlIn1/2: TB6612 方向控制引脚（IN1, IN2）。
 * Enc_TIMx: AB 相编码器定时器（编码器模式，TI12 4 倍频）。
 */
typedef struct {
    PWM_TypeDef_Struct   *PWMx;
    GPIO_TypeDef         *CtrlIn1_GPIOx;    /* TB6612 IN1 引脚端口 */
    uint16_t              CtrlIn1_GPIO_Pin; /* TB6612 IN1 引脚号 */
    GPIO_TypeDef         *CtrlIn2_GPIOx;    /* TB6612 IN2 引脚端口 */
    uint16_t              CtrlIn2_GPIO_Pin; /* TB6612 IN2 引脚号 */
    TIM_TypeDef          *Enc_TIMx;         /* 编码器定时器基地址 */
} MOTOR_hd_Struct;

/**
 * @brief 电机运行参数结构体
 *
 * speed: 当前滤波速度（脉冲/控制周期），由 EncUpdate 更新
 * speedMax/Min: 速度限幅值（signed），
 *   例如 Max=1.0 表示正转最大 100% PWM，Min=-1.0 表示反转最大 100% PWM
 * duty: 当前实际输出的 PWM 占空比（绝对值，[0,1]）
 * enc_*: 编码器 16→32 位扩展的状态变量（详见 MOTOR_EncUpdate）
 */
typedef struct {
    FunctionalState  enable;
    float            speed;
    float            speedMax;
    float            speedMin;
    float            duty;
    int32_t          enc_raw;        /* 32 位扩展累计脉冲 */
    int32_t          enc_prev;       /* 上一周期累计值（差分用） */
    float            enc_speed;      /* 低通滤波后速度 */
    uint16_t         enc_last_cnt;   /* 上次 16 位 CNT 原始值 */
} MOTOR_mem_Struct;

/**
 * @brief 电机合并句柄
 */
typedef struct {
    MOTOR_hd_Struct    hd;
    MOTOR_mem_Struct   mem;
} MOTOR_TypeDef_Struct;

/**
 * @brief 电机初始化配置结构体
 *
 * 注意：PWM 句柄在 Init 前单独设置（MOTOR1->hd.PWMx = PWM1），
 * 不在此结构体中，因为 PWM 是另一个模块的句柄，需要依赖注入。
 */
typedef struct {
    FunctionalState  enable;
    float            speedMax;       /* 正转最大速度 [0, 1] */
    float            speedMin;       /* 反转最大速度 [-1, 0] */
} MOTOR_InitTypeDef_Struct;

/*===========================================================================
 * 3. 全局实例池 + 宏别名
 *
 * 每个电机的引脚/编码器资源用 MOTOR_N_PROPERTY 命名，
 * 实例指针用 MOTORN 格式。
 *===========================================================================*/
#define MOTOR_BASE_NUM          2
extern MOTOR_TypeDef_Struct MOTOR_BASE[MOTOR_BASE_NUM];

/* MOTOR1: 左电机
 *   PWM  → PWM1 (TIM3_CH1 / PA6)
 *   方向 → PD9 (IN1), PD11 (IN2)
 *   编码 → TIM2 (PA0/PA1) */
#define MOTOR1                    (&MOTOR_BASE[0])
#define MOTOR1_CtrlIn1_GPIOx      GPIOD
#define MOTOR1_CtrlIn1_GPIO_Pin   GPIO_Pin_9
#define MOTOR1_CtrlIn2_GPIOx      GPIOD
#define MOTOR1_CtrlIn2_GPIO_Pin   GPIO_Pin_11
#define MOTOR1_Enc_TIMx           TIM2
#define MOTOR1_Enc_RCC_CLK        RCC_APB1Periph_TIM2
#define MOTOR1_Enc_GPIOx          GPIOA
#define MOTOR1_Enc_GPIO_Pin       (GPIO_Pin_0 | GPIO_Pin_1)

/* MOTOR2: 右电机
 *   PWM  → PWM2 (TIM3_CH2 / PA7)
 *   方向 → PD12 (IN1), PD10 (IN2)
 *   编码 → TIM4 (PB6/PB7) */
#define MOTOR2                    (&MOTOR_BASE[1])
#define MOTOR2_CtrlIn1_GPIOx      GPIOD
#define MOTOR2_CtrlIn1_GPIO_Pin   GPIO_Pin_12
#define MOTOR2_CtrlIn2_GPIOx      GPIOD
#define MOTOR2_CtrlIn2_GPIO_Pin   GPIO_Pin_10
#define MOTOR2_Enc_TIMx           TIM4
#define MOTOR2_Enc_RCC_CLK        RCC_APB1Periph_TIM4
#define MOTOR2_Enc_GPIOx          GPIOB
#define MOTOR2_Enc_GPIO_Pin       (GPIO_Pin_6 | GPIO_Pin_7)

/*===========================================================================
 * 4. API 函数声明
 *===========================================================================*/
ErrorStatus MOTOR_StructInit(MOTOR_InitTypeDef_Struct *p);
ErrorStatus MOTOR_Init(MOTOR_TypeDef_Struct *MOTORx, MOTOR_InitTypeDef_Struct *MOTOR_InitStruct);
ErrorStatus MOTOR_Enable(MOTOR_TypeDef_Struct *MOTORx);
ErrorStatus MOTOR_Disable(MOTOR_TypeDef_Struct *MOTORx);
ErrorStatus MOTOR_setSpeed(MOTOR_TypeDef_Struct *MOTORx, float speed);
ErrorStatus MOTOR_EncUpdate(MOTOR_TypeDef_Struct *MOTORx);
float      MOTOR_GetSpeed(MOTOR_TypeDef_Struct *MOTORx);
int32_t    MOTOR_GetEncCount(MOTOR_TypeDef_Struct *MOTORx);
ErrorStatus MOTOR_ResetEncCount(MOTOR_TypeDef_Struct *MOTORx);

#endif /* __MOTOR_H */
