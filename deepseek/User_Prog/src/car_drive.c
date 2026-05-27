/*******************************************************************************
 * @file    stm32f103_car_drive.c
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   小车控制模块实现：差速转向控制
 *
 * ── 完整控制链路（每 10ms）──
 *
 *   TRACKER_CalcError()         读取 8 路传感器 → 加权求和 → 位置误差
 *          │                    error ∈ [-1.0, +1.0]
 *          ▼
 *   PID_Compute(setpoint=0)     增量式 PID → 转向修正量
 *          │                    turn_output ∈ [-0.6, +0.6]
 *          ▼
 *   差速计算                     left  = basePWM + turn_output
 *                               right = basePWM - turn_output
 *          │                    限幅到 [0, speedMax]
 *          ▼
 *   MOTOR_setSpeed(L/R)         signed speed → TB6612 方向 + PWM 占空比
 *
 * ── 调参指南 ──
 *   1. 先调 basePWM：找到小车稳定直行的最低速度（通常 0.3~0.5）
 *   2. 由小到大调 Kp：直到小车能平滑归中，不震荡
 *   3. 如有过冲/震荡，加入 Kd 抑制（从 0.1 开始）
 *   4. Ki 通常保持 0（循迹不需要消除稳态位置误差）
 *
 * ── 丢线处理 ──
 *   默认策略（Tracker 层实现）：保持 lastValidError，小车惯性直行。
 *   如需紧急停车，调用 CARDRIVE_EmergencyStop()。
 *   如需回调搜索，可在 main.c task_10ms 中检测 IsLineDetected 后切换策略。
 *******************************************************************************/

#include "car_drive.h"
#include <stddef.h>

/*===========================================================================
 * 全局实例池
 *===========================================================================*/
CARDRIVE_TypeDef_Struct CARDRIVE_BASE[CARDRIVE_BASE_NUM];

/*===========================================================================
 * CARDRIVE_StructInit —— 用默认值填充 Init 结构体
 *
 * 默认 basePWM=0.4（40% 占空比），适合大多数桌面级循迹赛道。
 * 如果赛道摩擦力大或电池电量低，可适当提高此值。
 *===========================================================================*/
ErrorStatus CARDRIVE_StructInit(CARDRIVE_InitTypeDef_Struct *p)
{
    if(p == NULL)
        return ERROR;

    p->enable   = CARDRIVE_ENABLE_DEFAULT;
    p->basePWM  = CARDRIVE_BASEPWM_DEFAULT;
    p->speedMax = CARDRIVE_SPEEDMAX_DEFAULT;

    return SUCCESS;
}

/*===========================================================================
 * CARDRIVE_Init —— 初始化小车控制系统（五步法）
 *
 * 前置条件（依赖注入）：
 *   调用前必须设置 CARx->hd 的三个指针：
 *     CARx->hd.motor_L = MOTOR1;   // 已初始化的左电机
 *     CARx->hd.motor_R = MOTOR2;   // 已初始化的右电机
 *     CARx->hd.tracker = TRACKER1; // 已初始化的传感器
 *
 * PID 配置：
 *   使用位置式 PID（PID_MODE_POSITION），
 *   setpoint 固定为 0（居中），反馈为 tracker 误差。
 *   输出限幅 ±0.6 防止差速过大导致一侧车轮停转。
 *   例如 basePWM=0.4, turn=+0.6 → left=1.0, right=-0.2→0
 *   → 左轮全速 + 右轮停转 = 最强制左转。
 *===========================================================================*/
ErrorStatus CARDRIVE_Init(CARDRIVE_TypeDef_Struct *CARx, CARDRIVE_InitTypeDef_Struct *CARDRIVE_InitStruct)
{
    // 1. 定义局部变量
    ErrorStatus err = SUCCESS;

    // 2. 校验参数（含依赖注入完整性检查）
    if((CARx == NULL) || (CARDRIVE_InitStruct == NULL))
        return ERROR;
    if((CARx->hd.motor_L == NULL) || (CARx->hd.motor_R == NULL) || (CARx->hd.tracker == NULL))
        return ERROR;

    // 3. 配置参数
    CARx->mem.enable   = CARDRIVE_InitStruct->enable;
    CARx->mem.basePWM  = CARDRIVE_InitStruct->basePWM;
    CARx->mem.speedMax = CARDRIVE_InitStruct->speedMax;

    /* 初始化转向 PID 为增量式
     * 目标=0（居中），反馈=[-1,+1]（传感器误差），输出=[-0.6,+0.6]（PWM修正） */
    PID_Init(&CARx->pid_turn,
             PID_MODE_POSITION,
             CARDRIVE_KP_DEFAULT,
             CARDRIVE_KI_DEFAULT,
             CARDRIVE_KD_DEFAULT,
             CARDRIVE_PID_OUTMAX_DEFAULT,
            -CARDRIVE_PID_OUTMAX_DEFAULT);

    // 4. 初始化硬件（无额外硬件，下层 Motor/Tracker/PWM 已就绪）

    // 5. 执行命令：确保初始停止状态
    if(err == SUCCESS) {
        MOTOR_setSpeed(CARx->hd.motor_L, 0.0f);
        MOTOR_setSpeed(CARx->hd.motor_R, 0.0f);
    }

    return err;
}

/*===========================================================================
 * CARDRIVE_Enable —— 使能小车控制
 *
 * 设置软件使能标记。使能后，下一次 CARDRIVE_run() 才会执行控制循环。
 * 在使能前 CARDRIVE_run() 直接返回 ERROR。
 *===========================================================================*/
ErrorStatus CARDRIVE_Enable(CARDRIVE_TypeDef_Struct *CARx)
{
    if(CARx == NULL)
        return ERROR;

    CARx->mem.enable = ENABLE;
    return SUCCESS;
}

/*===========================================================================
 * CARDRIVE_Disable —— 禁能小车控制
 *
 * 停车 + 禁能标记。与 Enable 成对。
 * 禁能后 CARDRIVE_run() 不做任何操作，适用于暂停/紧急状态。
 *===========================================================================*/
ErrorStatus CARDRIVE_Disable(CARDRIVE_TypeDef_Struct *CARx)
{
    if(CARx == NULL)
        return ERROR;

    CARx->mem.enable = DISABLE;
    MOTOR_setSpeed(CARx->hd.motor_L, 0.0f);
    MOTOR_setSpeed(CARx->hd.motor_R, 0.0f);
    return SUCCESS;
}

/*===========================================================================
 * CARDRIVE_run —— 核心控制循环（每 10ms 由 scheduler 调用）
 *
 * ── 为什么放在一个函数里？ ──
 * 将"感知→决策→执行"封装为单一入口，调度器只需一行调用。
 * 如果需要切换控制策略（如：正常循迹 / 丢线搜索 / 避障），
 * 可以在 main.c 中创建多个 CARDRIVE 实例或切换 run 函数。
 *
 * ── 差速转向原理 ──
 * 小车转向靠左右轮速度差：
 *   - 偏右（error>0）→ PID输出>0 → 左轮加速 + 右轮减速 → 左转回中
 *   - 偏左（error<0）→ PID输出<0 → 左轮减速 + 右轮加速 → 右转回中
 *
 * ── 限幅的必要性 ──
 * basePWM + turn_output 可能超过 [0, 1] 范围：
 *   - >1.0: 超出 PWM 硬件能力，无效且可能损坏驱动
 *   - <0.0: 负 PWM 无意义（方向由 TB6612 IN1/IN2 控制，PWM 取绝对值）
 * 限幅到 [0, speedMax] 后再传给 MOTOR_setSpeed。
 *
 * ── 与原始设计的兼容性 ──
 * 本函数等价于原 control.c 的 Control_Run() + Tracker_CalcError() 组合，
 * 但遵循分层架构：CarDrive 不跨层直接调用 GPIO（传感器读取交给 Tracker），
 * 也不直接操作 TIM3 寄存器（PWM 输出交给 Motor→PWM）。
 *===========================================================================*/
ErrorStatus CARDRIVE_run(CARDRIVE_TypeDef_Struct *CARx)
{
    float tracker_error;
    float turn_output;
    float left_speed, right_speed;
    float abs_error, effective_base;

    if(CARx == NULL)
        return ERROR;
    if(CARx->mem.enable != ENABLE)
        return ERROR;

    /* 第一步：通过 Tracker 层读取传感器并计算位置误差
     * TRACKER_CalcError 内部完成 GPIO 读取 + 加权求和，返回 [-1.0, +1.0] */
    tracker_error = TRACKER_CalcError(CARx->hd.tracker);

    /* 第二步：转向 PID 计算修正量
     * 目标=0（黑线居中），反馈=当前误差，输出=左右轮速度修正值 */
    turn_output = PID_Compute(&CARx->pid_turn, 0.0f, tracker_error);

    /* 第三步：弯道减速 + 差速计算
     * |error|越大 → 弯越急 → 有效基础速度越低
     * 直道(|error|=0)全速，急弯(|error|=0.7)降至~65%速度 */
    abs_error = (tracker_error >= 0.0f) ? tracker_error : -tracker_error;
    effective_base = CARx->mem.basePWM
                         * (1.0f - CARDRIVE_CURVE_SLOWDOWN * abs_error);

    if(effective_base < 0.10f) effective_base = 0.10f;  /* 最低不低于10% */
    if(effective_base > CARx->mem.basePWM) effective_base = CARx->mem.basePWM;

    /* 左轮 = base + turn（正 turn → 左轮加速 → 左转）
     * 右轮 = base - turn（正 turn → 右轮减速 → 左转） */
    left_speed  = effective_base + turn_output;
    right_speed = effective_base - turn_output;

    /* 第四步：限幅到 [0, speedMax]
     * 下限 0 确保不输出负 PWM（方向由 MOTOR_setSpeed 内部处理）
     * 上限 speedMax 防止超速 */
    if(left_speed  > CARx->mem.speedMax) left_speed  = CARx->mem.speedMax;
    if(left_speed  < 0.0f)              left_speed  = 0.0f;
    if(right_speed > CARx->mem.speedMax) right_speed = CARx->mem.speedMax;
    if(right_speed < 0.0f)              right_speed = 0.0f;

    /* 第五步：通过 Motor 层输出到电机
     * MOTOR_setSpeed 内部处理 TB6612 方向 + 调用 PWM_setDuty */
    MOTOR_setSpeed(CARx->hd.motor_L,  left_speed);
    MOTOR_setSpeed(CARx->hd.motor_R,  right_speed);

    return SUCCESS;
}

/*===========================================================================
 * CARDRIVE_EmergencyStop —— 紧急停止
 *
 * 同时执行：
 *   1. 电机刹车（MOTOR_setSpeed 0=刹车，非惰行）
 *   2. PID 状态重置（清除积分累积和历史误差，防止重启后瞬间输出）
 *
 * 使用场景：丢线检测触发、碰撞检测、遥控急停。
 * 恢复运行：调用后直接调用 CARDRIVE_run() 即可，PID 从零开始重新收敛。
 *===========================================================================*/
ErrorStatus CARDRIVE_EmergencyStop(CARDRIVE_TypeDef_Struct *CARx)
{
    if(CARx == NULL)
        return ERROR;

    MOTOR_setSpeed(CARx->hd.motor_L, 0.0f);
    MOTOR_setSpeed(CARx->hd.motor_R, 0.0f);
    PID_Reset(&CARx->pid_turn);
    return SUCCESS;
}

/*===========================================================================
 * CARDRIVE_setBasePWM —— 设置基础速度（直行 PWM 占空比）
 *
 * 运行时动态调速接口。可用于：
 *   - 直道长直道 → 提高 basePWM 加速
 *   - 弯道/十字 → 降低 basePWM 减速
 *   - 电池电压下降 → 提高 basePWM 补偿
 *
 * 注意：输入被钳位到 [0, speedMax]，防止误设过大值。
 *===========================================================================*/
ErrorStatus CARDRIVE_setBasePWM(CARDRIVE_TypeDef_Struct *CARx, float pwm)
{
    if(CARx == NULL)
        return ERROR;

    if(pwm > CARx->mem.speedMax) pwm = CARx->mem.speedMax;
    if(pwm < 0.0f)              pwm = 0.0f;

    CARx->mem.basePWM = pwm;
    return SUCCESS;
}

/*===========================================================================
 * CARDRIVE_tunePID —— 在线调整转向 PID 参数
 *
 * 运行时调参接口，无需重新 Init。
 * 注意：大幅改变 Ki 后建议手动调用 PID_Reset() 清除历史积分。
 *===========================================================================*/
ErrorStatus CARDRIVE_tunePID(CARDRIVE_TypeDef_Struct *CARx, float kp, float ki, float kd)
{
    if(CARx == NULL)
        return ERROR;

    PID_Tune(&CARx->pid_turn, kp, ki, kd);
    return SUCCESS;
}
