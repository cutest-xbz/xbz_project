/*******************************************************************************
 * @file    main.c
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   智能小车 —— 主程序入口
 *
 * ── 软件架构（严格四层）──
 *
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  Layer 3: User_App/main.c                               │
 *   │           系统初始化 + 调度器 + 主循环                    │
 *   │           只调用 CarDrive（绝不跨层调 Motor/Tracker）     │
 *   ├─────────────────────────────────────────────────────────┤
 *   │  Layer 2: User_Prog/  应用组合层                          │
 *   │   ├── car_drive.c      小车差速转向控制                   │
 *   │   │   注入: Motor×2 + Tracker×1                          │
 *   │   ├── motor.c          电机方向 + 编码器                  │
 *   │   │   注入: PWM×2                                        │
 *   │   └── pid.c            增量式/位置式 PID 算法             │
 *   ├─────────────────────────────────────────────────────────┤
 *   │  Layer 1: User_Driver/  硬件抽象层                        │
 *   │   ├── pwm.c            TIM3 CH1/CH2 10kHz PWM           │
 *   │   └── tracker.c        GPIOD PD0~PD7 8路灰度传感器       │
 *   ├─────────────────────────────────────────────────────────┤
 *   │  Layer 0: FWlib/       STM32 标准外设库 (SPL v3.5)       │
 *   └─────────────────────────────────────────────────────────┘
 *
 * ── 铁律：上层只调紧邻下层，绝不跨层 ──
 *   main.c    → 只调 CarDrive API
 *   CarDrive  → 只调 Motor + Tracker API
 *   Motor     → 只调 PWM API
 *   PWM       → 只调 SPL (TIM/GPIO/RCC)
 *   Tracker   → 只调 SPL (GPIO)
 *
 * ── 初始化序列（自底向上 + 依赖注入）──
 *   1. 时钟系统 (RCC 72MHz + SysTick 1ms)
 *   2. PWM 层 (PWM1/PWM2)
 *   3. Tracker 层 (TRACKER1)
 *   4. Motor 层 (MOTOR1/MOTOR2, 注入 PWM1/PWM2)
 *   5. CarDrive 层 (CAR1, 注入 MOTOR1/MOTOR2/TRACKER1)
 *   6. 调度器注册 + 启动
 *
 * ── 控制周期（10ms）──
 *   SysTick 1ms → scheduler → task_10ms() → CARDRIVE_run(CAR1)
 *   CARDRIVE_run 内部完成完整的 感知→决策→执行 链路。
 *
 * 硬件平台：STM32F103ZE + TB6612 + 8路红外 + AB相编码器×2
 * 开发库：  STM32 标准外设库 (SPL v3.5)
 * 主频：    72MHz（HSE 8MHz → PLL ×9）
 *******************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stddef.h>
#include "scheduler.h"
#include "device_timer.h"
#include "pwm.h"
#include "tracker.h"
#include "motor.h"
#include "car_drive.h"

/*===========================================================================
 * 前向声明
 *===========================================================================*/
void RCC_Configuration(void);

/*===========================================================================
 * 任务函数（由调度器按周期回调）
 *
 * 每个任务函数对应一个调度周期。当前只使用 task_10ms（核心控制循环），
 * 其余为预留扩展接口。
 *===========================================================================*/

/**
 * @brief 1ms 周期任务（最高频）
 * 【预留】急停检测、编码器更新、通信接收等对实时性要求最高的操作。
 */
void task_1ms(void)
{
}

/**
 * @brief 5ms 周期任务
 * 【预留】传感器预处理、状态机更新等中频操作。
 */
void task_5ms(void)
{
}

/**
 * @brief 10ms 周期任务 —— 核心控制循环
 *
 * 这是整个系统唯一需要周期性执行的业务逻辑。
 * CARDRIVE_run() 内部完成：
 *   1. 传感器读取 + 位置误差计算 (Tracker)
 *   2. 增量式 PID 转向修正 (PID)
 *   3. 差速计算 + 限幅
 *   4. 电机输出 (Motor → PWM)
 *
 * 控制周期选择 10ms 的原因：
 *   - 够快：100Hz 对小车机械响应足够（电机惯性 > 20ms）
 *   - 够稳：留足裕量给其他任务（1ms/5ms/20ms 等）
 *   - 省资源：不超过 SysTick(1ms) × 10，调度精准
 */
void task_10ms(void)
{
    CARDRIVE_run(CAR1);
}

/**
 * @brief 20ms 周期任务
 * 【预留】OLED 显示刷新、状态上报等。
 */
void task_20ms(void)
{
}

/**
 * @brief 50ms 周期任务
 * 【预留】调试信息输出（串口打印速度、误差等）。
 *
 * 调试代码示例（取消注释以启用）：
 *
 *   static uint16_t dbg_cnt = 0;
 *   if (++dbg_cnt >= 5) {         // 每 250ms 输出一次
 *       dbg_cnt = 0;
 *       float spdL = MOTOR_GetSpeed(MOTOR1);
 *       float spdR = MOTOR_GetSpeed(MOTOR2);
 *       // printf("L:%.2f  R:%.2f\r\n", spdL, spdR);
 *   }
 */
void task_50ms(void)
{
}

/**
 * @brief 100ms 周期任务（最低频）
 * 【预留】LED 闪烁、电池电压检测、模式切换等低速任务。
 */
void task_100ms(void)
{
}

/*===========================================================================
 * 主函数 —— 系统入口
 *
 * 初始化严格遵循自底向上顺序：
 *   时钟 → PWM → Tracker → Motor（注入PWM） → CarDrive（注入Motor+Tracker）
 * 这种顺序确保上层 Init 时，下层实例已经就绪。
 *===========================================================================*/
int main(void)
{
    // 1. 声明所有需要的变量（集中在顶部，Allman 风格）
    SCHEDULER_TASKS_Struct       allTasks;
    PWM_InitTypeDef_Struct       pwmInitStruct;
    TRACKER_InitTypeDef_Struct   trackerInitStruct;
    MOTOR_InitTypeDef_Struct     motorInitStruct;
    CARDRIVE_InitTypeDef_Struct  carInitStruct;

    // 2. 硬件初始化（自底向上：时钟 → SysTick）
    RCC_Configuration();          /* HSE 8MHz → PLL ×9 → SYSCLK 72MHz */
    SysTick_Configuration();      /* 1ms 中断，驱动调度器 */

    // 3. 外设初始化（StructInit → 注入依赖 → Init，逐层向上）

    /* =================================================================
     * 3.1 PWM 层（Layer 1 硬件抽象层 —— 最底层）
     *
     * PWM1: TIM3_CH1 (PA6) → 左电机速度
     * PWM2: TIM3_CH2 (PA7) → 右电机速度
     * 两通道共享 TIM3 时基（10kHz），各自独立控制占空比。
     * ================================================================= */
    PWM_StructInit(&pwmInitStruct);     /* 填充默认值：10kHz, 0%占空比 */
    PWM_Init(PWM1, &pwmInitStruct);     /* 绑定 TIM3_CH1 + PA6，使能输出 */
    PWM_Init(PWM2, &pwmInitStruct);     /* 绑定 TIM3_CH2 + PA7，使能输出 */

    /* =================================================================
     * 3.2 Tracker 层（Layer 1 硬件抽象层 —— 最底层）
     *
     * TRACKER1: GPIOD PD0~PD7 → 8路红外传感器
     * 8 路传感器一字排列，权重左负右正用于加权求和位置计算。
     * ================================================================= */
    TRACKER_StructInit(&trackerInitStruct);
    TRACKER_Init(TRACKER1, &trackerInitStruct);

    /* =================================================================
     * 3.3 Motor 层（Layer 2 应用层 —— 依赖 PWM）
     *
     * MOTOR1(左): PWM1 + PD9/PD11(TB6612) + TIM2编码器(PA0/PA1)
     * MOTOR2(右): PWM2 + PD12/PD10(TB6612) + TIM4编码器(PB6/PB7)
     *
     * 依赖注入：在 Init 前设置 hd.PWMx，Motor 通过句柄调 PWM API。
     * 这确保了 Motor 不直接访问 PWM_BASE[] 全局变量。
     * ================================================================= */
    MOTOR_StructInit(&motorInitStruct);
    MOTOR1->hd.PWMx = PWM1;             /* 依赖注入：左电机 → PWM1 */
    MOTOR_Init(MOTOR1, &motorInitStruct);
    MOTOR2->hd.PWMx = PWM2;             /* 依赖注入：右电机 → PWM2 */
    MOTOR_Init(MOTOR2, &motorInitStruct);

    /* =================================================================
     * 3.4 CarDrive 层（Layer 2 应用组合层 —— 依赖 Motor + Tracker）
     *
     * CAR1: 整合 MOTOR1/2 + TRACKER1 + 转向PID
     *
     * 依赖注入：在 Init 前设置三个句柄指针。
     * CarDrive 是 main.c 唯一直接调用的业务模块。
     * ================================================================= */
    CARDRIVE_StructInit(&carInitStruct);
    CAR1->hd.motor_L = MOTOR1;          /* 依赖注入：左电机 */
    CAR1->hd.motor_R = MOTOR2;          /* 依赖注入：右电机 */
    CAR1->hd.tracker = TRACKER1;        /* 依赖注入：灰度传感器 */
    CARDRIVE_Init(CAR1, &carInitStruct);

    /* =================================================================
     * 4. 注册调度任务
     *
     * 只注册实际使用的任务（10ms），其余置 NULL 由调度器跳过。
     * 调度器由 SysTick 1ms 中断驱动，各任务按预设周期自动执行。
     * ================================================================= */
    allTasks.pDuty_1ms   = task_1ms;
    allTasks.pDuty_2ms   = NULL;
    allTasks.pDuty_3ms   = NULL;
    allTasks.pDuty_4ms   = NULL;
    allTasks.pDuty_5ms   = task_5ms;
    allTasks.pDuty_10ms  = task_10ms;   /* 核心：每 10ms 执行一次控制循环 */
    allTasks.pDuty_20ms  = task_20ms;
    allTasks.pDuty_50ms  = task_50ms;
    allTasks.pDuty_100ms = task_100ms;

    Scheduler_Init(allTasks);

    /* =================================================================
     * 5. 启动调度器，进入主循环
     *
     * Scheduler_Start() 注册 SysTick 中断钩子。
     * 此后 SysTick 每 1ms 触发一次 Scheduler_Loop_timer()，
     * 主循环不断检查调度标志并执行到期任务。
     * ================================================================= */
    Scheduler_Start();
    while(1) {
        Scheduler_Main_Loop();
    }
}

/*===========================================================================
 * 系统时钟配置
 *
 * 使用 HSE 外部晶振（8MHz）：
 *   STM32F103ZE (LQFP144) 上 OSC_IN=PH0, OSC_OUT=PH1 是独立引脚，
 *   不占用 PD0/PD1。因此可以同时使用 HSE 高速外部晶振和 GPIOD 全 16 路。
 *   HSE 精度远优于 HSI（±1% → ±0.001%），72MHz 是 F103 标称主频。
 *
 * 时钟树（从 HSE 到各总线）：
 *   HSE (8MHz)
 *     └→ /1 (8MHz, PLLXTPRE=0)
 *          └→ PLL ×9 → SYSCLK (72MHz)
 *                           ├→ HCLK  = 72MHz (AHB: Flash, DMA, SRAM)
 *                           ├→ PCLK2 = 72MHz (APB2: GPIOA~G, AFIO, TIM1)
 *                           └→ PCLK1 = 36MHz (APB1: TIM2/3/4)
 *                                └→ TIMx_CLK = PCLK1×2 = 72MHz
 *                                   （APB1 预分频≠1 时，定时器时钟自动×2）
 *
 * Flash 等待周期：
 *   72MHz 需要 2 个等待周期（FLASH_Latency_2）。
 *   如果跳过此设置，Flash 读取可能出错导致 HardFault。
 *===========================================================================*/
void RCC_Configuration(void)
{
    /* 复位所有 RCC 寄存器为默认值（HSI 自动使能） */
    RCC_DeInit();

    /* 使能 HSE 外部晶振，等待就绪 */
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) {}

    /* Flash 配置：预取缓冲 + 2 等待周期（72MHz 必需） */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);

    /* AHB = SYSCLK / 1 = 72MHz */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* APB2 = HCLK / 1 = 72MHz（GPIO, AFIO, TIM1 等高速外设） */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* APB1 = HCLK / 2 = 36MHz
     * 注意：TIM2/3/4 在 APB1，当 APB1 预分频≠1 时，
     * 定时器时钟自动 ×2 = 72MHz。这是硬件自动行为，无需软件配置。 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLL: HSE/1=8MHz → ×9 → 72MHz
     * RCC_PLLSource_HSE_Div1: HSE 不分频直接作为 PLL 输入
     * RCC_PLLMul_9: PLL 倍频系数 9 倍 */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

    /* 切换系统时钟源为 PLL */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08) {}  /* 等待 SWS=10(0x08) 确认切换完成 */

    /* 全局使能所有 GPIO 端口时钟
     * 各模块内部也会使能所需时钟，此处是防御性全局使能。
     * 重复使能已使能的时钟无副作用。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);

    /* 全局使能所有定时器时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  /* 左编码器 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  /* PWM */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  /* 右编码器 */
}

/*===========================================================================
 * 异常处理 —— HardFault
 *
 * 硬件错误（非法内存访问、未对齐访问、总线错误等）的最终兜底。
 * 策略：关全局中断 + 死循环，防止程序跑飞。
 * 调试时可在此处加 LED 闪烁或串口报错。
 *===========================================================================*/
void HardFault_Handler(void)
{
    __disable_irq();
    while (1) {}
}
