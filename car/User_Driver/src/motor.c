/*******************************************************************************
 * @file    stm32f103_motor.c
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   电机模块实现：TB6612 方向控制 + AB 相编码器读取
 *
 * ── TB6612 驱动原理 ──
 * TB6612 是双路 H 桥电机驱动芯片，每路有 3 个控制信号：
 *   IN1, IN2: 方向控制（4 种状态）
 *   PWM:      速度控制（占空比 = 转速比例）
 *
 *   真值表（以 A 通道为例）：
 *     IN1=H, IN2=L, PWM  → 正转（电流 A→B）
 *     IN1=L, IN2=H, PWM  → 反转（电流 B→A）
 *     IN1=L, IN2=L       → 惰行/停止（H 桥高阻，电机惯性转动）
 *     IN1=H, IN2=H       → 短接刹车（H 桥短路制动，电机快速停止）
 *
 * ── 编码器原理 ──
 * AB 相增量式编码器输出两路相位差 90° 的方波。
 * STM32 定时器编码器模式自动计数：
 *   TI1+TI2 双边沿 = 4 倍频（每个物理脉冲计 4 个数）
 *   正转 → CNT 递增，反转 → CNT 递减
 * 16→32 位扩展解决 65535 溢出问题，详见 MOTOR_EncUpdate 注释。
 *
 * ── 共享资源处理 ──
 * 两个电机的方向引脚都在 GPIOD 上。使用静态标志 motor_gpiod_inited
 * 确保 GPIOD 时钟只使能一次。编码器使用不同的定时器（TIM2/TIM4），互不冲突。
 *******************************************************************************/

#include "motor.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include <math.h>
#include <stddef.h>

/*===========================================================================
 * 全局实例池
 *===========================================================================*/
MOTOR_TypeDef_Struct MOTOR_BASE[MOTOR_BASE_NUM];

/**
 * @brief GPIOD 方向引脚时钟使能标志
 *
 * MOTOR1 和 MOTOR2 的方向引脚都在 GPIOD 上。
 * 第一个 Init 的电机负责使能 GPIOD 时钟并置标志。
 */
static uint8_t motor_gpiod_inited = 0;

/*===========================================================================
 * MOTOR_StructInit —— 用默认值填充 Init 结构体
 *
 * 将速度限幅设为默认值 Max=1.0(100%正转), Min=-1.0(100%反转)。
 * 用户可随后覆盖 speedMax/speedMin 以限制电机出力。
 *===========================================================================*/
ErrorStatus MOTOR_StructInit(MOTOR_InitTypeDef_Struct *p)
{
    if(p == NULL)
        return ERROR;

    p->enable   = MOTOR_ENABLE_DEFAULT;
    p->speedMax = MOTOR_SPEED_MAX;
    p->speedMin = MOTOR_SPEED_MIN;

    return SUCCESS;
}

/*===========================================================================
 * MOTOR_Init —— 初始化电机（五步法）
 *
 * 前置条件：在调用本函数前，上层必须设置 MOTORx->hd.PWMx 指针。
 *   例如：MOTOR1->hd.PWMx = PWM1;  // 依赖注入
 *
 * 初始化内容：
 *   1. 绑定 TB6612 方向控制引脚（GPIO 推挽输出，初始低电平=停止）
 *   2. 配置编码器定时器为编码器接口模式（TI12 4 倍频）
 *   3. 清零所有软件状态变量
 *   4. 在 enable==ENABLE 时调用 MOTOR_setSpeed(0) 确保初始刹车
 *
 * 编码器配置细节：
 *   TIM_EncoderMode_TI12: CH1 和 CH2 都作为编码器输入，双边沿触发
 *   → 每个物理脉冲产生 4 个计数（A↑, A↓, B↑, B↓）
 *   TIM_ICPolarity_Rising: 不反相输入信号
 *   ARR=65535: 最大化计数范围（16 位满量程）
 *
 * 为什么不在 Init 中调用 MOTOR_EncUpdate？
 *   EncUpdate 需要在控制周期中定期调用，Init 只做初始化。
 *   编码器在 TIM_Cmd(ENABLE) 后自动开始计数。
 *===========================================================================*/
ErrorStatus MOTOR_Init(MOTOR_TypeDef_Struct *MOTORx, MOTOR_InitTypeDef_Struct *MOTOR_InitStruct)
{
    // 1. 定义局部变量
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
    GPIO_InitTypeDef         GPIO_InitStructure;
    ErrorStatus              err = SUCCESS;

    // 2. 校验参数（含依赖注入检查）
    if((MOTORx == NULL) || (MOTOR_InitStruct == NULL))
        return ERROR;
    if(MOTORx->hd.PWMx == NULL)     /* PWM 句柄未注入！ */
        return ERROR;

    // 3. 配置参数（硬件绑定 + 参数拷贝）
    if(MOTORx == MOTOR1) {
        MOTOR_BASE[0].hd.CtrlIn1_GPIOx     = MOTOR1_CtrlIn1_GPIOx;
        MOTOR_BASE[0].hd.CtrlIn1_GPIO_Pin  = MOTOR1_CtrlIn1_GPIO_Pin;
        MOTOR_BASE[0].hd.CtrlIn2_GPIOx     = MOTOR1_CtrlIn2_GPIOx;
        MOTOR_BASE[0].hd.CtrlIn2_GPIO_Pin  = MOTOR1_CtrlIn2_GPIO_Pin;
        MOTOR_BASE[0].hd.Enc_TIMx          = MOTOR1_Enc_TIMx;
    } else if(MOTORx == MOTOR2) {
        MOTOR_BASE[1].hd.CtrlIn1_GPIOx     = MOTOR2_CtrlIn1_GPIOx;
        MOTOR_BASE[1].hd.CtrlIn1_GPIO_Pin  = MOTOR2_CtrlIn1_GPIO_Pin;
        MOTOR_BASE[1].hd.CtrlIn2_GPIOx     = MOTOR2_CtrlIn2_GPIOx;
        MOTOR_BASE[1].hd.CtrlIn2_GPIO_Pin  = MOTOR2_CtrlIn2_GPIO_Pin;
        MOTOR_BASE[1].hd.Enc_TIMx          = MOTOR2_Enc_TIMx;
    } else {
        return ERROR;
    }

    MOTORx->mem.enable    = MOTOR_InitStruct->enable;
    MOTORx->mem.speedMax  = MOTOR_InitStruct->speedMax;
    MOTORx->mem.speedMin  = MOTOR_InitStruct->speedMin;
    MOTORx->mem.speed     = 0.0f;
    MOTORx->mem.duty      = 0.0f;
    MOTORx->mem.enc_raw   = 0;
    MOTORx->mem.enc_prev  = 0;
    MOTORx->mem.enc_speed = 0.0f;
    MOTORx->mem.enc_last_cnt = 0;

    // 4. 初始化硬件
    if(err == SUCCESS) {
        // 4.1 初始化方向控制 GPIO（推挽输出，初始输出低电平=停止）
        //     GPIOD 时钟仅使能一次（两个电机共享 GPIOD）
        if(!motor_gpiod_inited) {
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
            motor_gpiod_inited = 1;
        }

        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin   = MOTORx->hd.CtrlIn1_GPIO_Pin
                                      | MOTORx->hd.CtrlIn2_GPIO_Pin;
        GPIO_Init(MOTORx->hd.CtrlIn1_GPIOx, &GPIO_InitStructure);
        GPIO_ResetBits(MOTORx->hd.CtrlIn1_GPIOx,
                       MOTORx->hd.CtrlIn1_GPIO_Pin | MOTORx->hd.CtrlIn2_GPIO_Pin);

        // 4.2 初始化编码器定时器（编码器模式 + 浮空输入）
        //     左编码器 TIM2 用 PA0/PA1，右编码器 TIM4 用 PB6/PB7
        if(MOTORx == MOTOR1) {
            RCC_APB1PeriphClockCmd(MOTOR1_Enc_RCC_CLK, ENABLE);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_InitStructure.GPIO_Pin  = MOTOR1_Enc_GPIO_Pin;
            GPIO_Init(MOTOR1_Enc_GPIOx, &GPIO_InitStructure);
        } else {
            RCC_APB1PeriphClockCmd(MOTOR2_Enc_RCC_CLK, ENABLE);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_InitStructure.GPIO_Pin  = MOTOR2_Enc_GPIO_Pin;
            GPIO_Init(MOTOR2_Enc_GPIOx, &GPIO_InitStructure);
        }

        /* 编码器时基：计数范围 0~65535（16 位满量程） */
        TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
        TIM_TimeBaseInitStruct.TIM_Period        = 65535;
        TIM_TimeBaseInitStruct.TIM_Prescaler     = 0;
        TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;
        TIM_TimeBaseInit(MOTORx->hd.Enc_TIMx, &TIM_TimeBaseInitStruct);

        /* 配置为编码器模式：TI1+TI2 双边沿 = 4 倍频 */
        TIM_EncoderInterfaceConfig(MOTORx->hd.Enc_TIMx,
            TIM_EncoderMode_TI12,
            TIM_ICPolarity_Rising,
            TIM_ICPolarity_Rising);
    }

    // 5. 执行命令：启动编码器计数 + 初始刹车确保安全
    if(err == SUCCESS) {
        TIM_Cmd(MOTORx->hd.Enc_TIMx, ENABLE);
        if(MOTORx->mem.enable == ENABLE) {
            MOTOR_setSpeed(MOTORx, 0.0f);   /* 初始刹车 */
        }
    }

    return err;
}

/*===========================================================================
 * MOTOR_Enable —— 使能电机（软件标记）
 *
 * 注意：不自动驱动电机，仅清除软件禁止标记。
 * 使能后下一个控制周期才会输出速度。
 *===========================================================================*/
ErrorStatus MOTOR_Enable(MOTOR_TypeDef_Struct *MOTORx)
{
    if(MOTORx == NULL)
        return ERROR;

    MOTORx->mem.enable = ENABLE;
    return SUCCESS;
}

/*===========================================================================
 * MOTOR_Disable —— 禁能电机（惰行停止 + 清除软件标记）
 *
 * 与 Enable 成对。禁能时输出 PWM=0 且方向引脚拉低，
 * 电机进入惰行状态（H 桥高阻，无制动力）。
 *===========================================================================*/
ErrorStatus MOTOR_Disable(MOTOR_TypeDef_Struct *MOTORx)
{
    if(MOTORx == NULL)
        return ERROR;

    MOTORx->mem.enable = DISABLE;
    PWM_setDuty(MOTORx->hd.PWMx, 0.0f);
    GPIO_ResetBits(MOTORx->hd.CtrlIn1_GPIOx,
                   MOTORx->hd.CtrlIn1_GPIO_Pin | MOTORx->hd.CtrlIn2_GPIO_Pin);
    return SUCCESS;
}

/*===========================================================================
 * MOTOR_setSpeed —— 设置电机速度（signed speed 统一接口）
 *
 * ── signed speed 设计 ──
 * 传统电机接口：Motor_Set(dir, duty) —— 方向和占空比分离
 * signed speed 接口：Motor_setSpeed(speed) —— 符号表方向，绝对值表大小
 *
 * 优势：
 *   1. 天然适配 PID 输出（PID 输出本身是有符号修正量）
 *   2. 调用者不需要判断"应该正转还是反转"
 *   3. 差速转向计算更简洁：left=base+turn, right=base-turn
 *
 * ── 死区处理 ──
 * |speed| < 0.001 视为零，执行刹车（IN1=H, IN2=H）。
 * 刹车比惰行停止更快，适合循迹小车的快速纠偏。
 * 死区阈值 0.001 防止因浮点舍入误差导致的电机微动/啸叫。
 *
 * ── 实现步骤 ──
 * 1. 限幅 speed 到 [speedMin, speedMax]
 * 2. 根据符号设置 TB6612 方向引脚（正=IN1_H_IN2_L, 负=IN1_L_IN2_H）
 * 3. 调用下层 PWM_setDuty() 设置占空比（取 speed 绝对值）
 *===========================================================================*/
ErrorStatus MOTOR_setSpeed(MOTOR_TypeDef_Struct *MOTORx, float speed)
{
    float fspeed;

    if(MOTORx == NULL)
        return ERROR;

    /* 限幅到 [speedMin, speedMax] */
    fspeed = speed;
    if(fspeed > MOTORx->mem.speedMax)  fspeed = MOTORx->mem.speedMax;
    if(fspeed < MOTORx->mem.speedMin)  fspeed = MOTORx->mem.speedMin;

    MOTORx->mem.speed = fspeed;

    /* 根据符号执行 TB6612 方向 + PWM 输出 */
    if(fspeed > 0.001f) {
        /* 正转：IN1=H, IN2=L */
        GPIO_SetBits(MOTORx->hd.CtrlIn1_GPIOx,  MOTORx->hd.CtrlIn1_GPIO_Pin);
        GPIO_ResetBits(MOTORx->hd.CtrlIn2_GPIOx, MOTORx->hd.CtrlIn2_GPIO_Pin);
        PWM_setDuty(MOTORx->hd.PWMx, fspeed);
    } else if(fspeed < -0.001f) {
        /* 反转：IN1=L, IN2=H */
        GPIO_ResetBits(MOTORx->hd.CtrlIn1_GPIOx, MOTORx->hd.CtrlIn1_GPIO_Pin);
        GPIO_SetBits(MOTORx->hd.CtrlIn2_GPIOx,   MOTORx->hd.CtrlIn2_GPIO_Pin);
        PWM_setDuty(MOTORx->hd.PWMx, -fspeed);
    } else {
        /* 刹车：IN1=H, IN2=H（H 桥下管导通，电机绕组短路制动） */
        GPIO_SetBits(MOTORx->hd.CtrlIn1_GPIOx, MOTORx->hd.CtrlIn1_GPIO_Pin);
        GPIO_SetBits(MOTORx->hd.CtrlIn2_GPIOx, MOTORx->hd.CtrlIn2_GPIO_Pin);
        PWM_setDuty(MOTORx->hd.PWMx, 0.0f);
    }

    MOTORx->mem.duty = (fspeed >= 0.0f) ? fspeed : -fspeed;
    return SUCCESS;
}

/*===========================================================================
 * MOTOR_EncUpdate —— 读取并更新编码器数据（每控制周期调用一次）
 *
 * ── 为什么需要 16→32 位扩展？ ──
 * STM32F103 的定时器是 16 位，CNT 范围 [0, 65535]。
 * 小车跑几秒就会溢出回绕，直接使用 16 位值无法正确追踪总位移。
 * 本函数通过累计差值将 16 位硬件计数扩展到 32 位（±21 亿），
 * 足以支持连续运行数十小时。
 *
 * ── 溢出处理原理（关键技巧）──
 * 假设连续两次读取：
 *   上次 CNT = 65530
 *   本次 CNT = 5
 * 直接相减：(uint16_t)(5 - 65530) = 11（无符号溢出，错误！）
 *
 * 转为 int16_t 再相减：
 *   delta = (int16_t)(5 - 65530) = (int16_t)(11) = 11  ✓ 正确！
 *
 * 原理：无符号减法溢出的结果，恰好等于有符号差值。
 * 只要相邻两次读取的间隔内 CNT 变化不超过 ±32767，
 * int16_t 差值就能正确反映实际的脉冲增量和方向。
 * （以 1560 脉冲/圈 @ 10ms 周期，需要 >200 rps 才会超限，实际远不至于）
 *
 * ── 一阶低通滤波 ──
 * filtered = α × old_filtered + (1-α) × raw_speed
 * α = MOTOR_ENC_FILTER = 0.3
 * 滤波作用：消除编码器量化噪声和机械振动引起的高频波动。
 * 0.3 的选取经验：在 10ms 控制周期下，速度变化平滑且不失响应速度。
 *===========================================================================*/
ErrorStatus MOTOR_EncUpdate(MOTOR_TypeDef_Struct *MOTORx)
{
    uint16_t cnt_now;
    int16_t  delta;
    float    raw_speed;

    if(MOTORx == NULL)
        return ERROR;

    /* 步骤1：读取当前硬件 CNT（16 位） */
    cnt_now = MOTORx->hd.Enc_TIMx->CNT;

    /* 步骤2：计算与上次的 16 位差值（int16_t 自动处理溢出） */
    delta = (int16_t)(cnt_now - MOTORx->mem.enc_last_cnt);
    MOTORx->mem.enc_last_cnt = cnt_now;

    /* 步骤3：累加到 32 位扩展计数器 */
    MOTORx->mem.enc_raw += (int32_t)delta;

    /* 步骤4：差分 32 位计数器得到本周期脉冲增量（原始速度） */
    raw_speed = (float)(MOTORx->mem.enc_raw - MOTORx->mem.enc_prev);
    MOTORx->mem.enc_prev = MOTORx->mem.enc_raw;

    /* 步骤5：一阶低通滤波平滑速度 */
    MOTORx->mem.enc_speed = MOTORx->mem.enc_speed * MOTOR_ENC_FILTER
                          + raw_speed * (1.0f - MOTOR_ENC_FILTER);

    MOTORx->mem.speed = MOTORx->mem.enc_speed;
    return SUCCESS;
}

/*===========================================================================
 * MOTOR_GetSpeed —— 获取当前低通滤波后的速度
 *
 * @return 速度（脉冲/控制周期），正值=正转，负值=反转
 *===========================================================================*/
float MOTOR_GetSpeed(MOTOR_TypeDef_Struct *MOTORx)
{
    if(MOTORx == NULL)
        return 0.0f;

    return MOTORx->mem.enc_speed;
}

/*===========================================================================
 * MOTOR_GetEncCount —— 获取 32 位扩展累计脉冲数
 *
 * 可用于计算总行驶距离：distance = count / (PPR × 4) × π × wheel_diameter
 * @return 累计脉冲数（带符号，正=正向移动）
 *===========================================================================*/
int32_t MOTOR_GetEncCount(MOTOR_TypeDef_Struct *MOTORx)
{
    if(MOTORx == NULL)
        return 0;

    return MOTORx->mem.enc_raw;
}

/*===========================================================================
 * MOTOR_ResetEncCount —— 清零编码器累计计数
 *
 * 用于里程归零（如起点标定、分段测距）。
 * 注意：同时清零 enc_raw 和 enc_prev，保持两者一致避免差分跳变。
 *===========================================================================*/
ErrorStatus MOTOR_ResetEncCount(MOTOR_TypeDef_Struct *MOTORx)
{
    if(MOTORx == NULL)
        return ERROR;

    MOTORx->mem.enc_raw  = 0;
    MOTORx->mem.enc_prev = 0;
    return SUCCESS;
}
