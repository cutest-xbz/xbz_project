/*******************************************************************************
 * @file    stm32f103_pwm.c
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   PWM 模块实现：TIM3 双通道 PWM 输出
 *
 * 硬件细节：
 *   - 定时器时钟：APB1 × 2 = 72MHz（来自 main.c RCC 配置）
 *   - PWM 频率 = TIM_CLK / (PSC+1) / (ARR+1)
 *              = 72MHz / 1 / 7200 = 10kHz（使用默认 freq=10000Hz）
 *   - 分辨率 = ARR+1 = 6400 步
 *   - 两个通道共享 TIM3 时基，但各自独立控制 CCR1/CCR2
 *
 * 共享资源处理：
 *   PWM1 和 PWM2 共用 TIM3，其 ARR/PSC 由第一个调用 Init 的实例设定。
 *   使用静态标志 pwm_tim3_inited 确保 TIM3 时基只初始化一次，
 *   避免后初始化的实例覆盖前者的时基配置。
 *   GPIOA 时钟使能是幂等操作（重复使能无副作用）。
 *******************************************************************************/

#include "pwm.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include <stddef.h>

/*===========================================================================
 * 全局实例池
 *
 * 编译时分配 2 个 PWM 实例的内存空间。
 * 每个实例独立维护自己的 hd（硬件绑定）和 para（运行参数）。
 * 上层通过 PWM1/PWM2 宏别名访问。
 *===========================================================================*/
PWM_TypeDef_Struct PWM_BASE[PWM_BASE_NUM];

/**
 * @brief TIM3 时基初始化标志
 *
 * PWM1 和 PWM2 共享同一定时器 TIM3。ARR/PSC 等时基参数只需配置一次。
 * 第一个 Init 的通道负责配时基并置此标志为 1；
 * 后续通道只初始化自己的 OC 通道和 GPIO，不再触碰时基。
 */
static uint8_t pwm_tim3_inited = 0;

/*===========================================================================
 * PWM_StructInit —— 用默认值填充 Init 结构体
 *
 * 纯数据填充，无任何硬件操作或额外逻辑。
 * 调用时机：上电后、PWM_Init() 之前。
 * 典型用法：
 *   PWM_InitTypeDef_Struct cfg;
 *   PWM_StructInit(&cfg);          // 全部填默认值
 *   cfg.freq = 20000.0f;           // 按需覆盖频率
 *   cfg.duty = 0.5f;               // 按需覆盖初始占空比
 *   PWM_Init(PWM1, &cfg);          // 传入 Init
 *===========================================================================*/
ErrorStatus PWM_StructInit(PWM_InitTypeDef_Struct *p)
{
    if(p == NULL)
        return ERROR;

    p->enable  = PWM_ENABLE_DEFAULT;
    p->log     = PWM_LOG_DEFAULT;
    p->freq    = PWM_FREQ_DEFAULT;
    p->dutyMax = PWM_DUTYMAX_DEFAULT;
    p->dutyMin = PWM_DUTYMIN_DEFAULT;
    p->duty    = PWM_DUTY_DEFAULT;

    return SUCCESS;
}

/*===========================================================================
 * PWM_Init —— 完整初始化一个 PWM 通道（五步法）
 *
 * 五步模板：
 *   1. 定义局部变量 —— 所有 SPL 库结构体 + ErrorStatus err
 *   2. 校验参数     —— 检查指针非空，检查宏别名合法
 *   3. 配置参数     —— 根据宏别名绑定硬件 + 拷贝 InitStruct → para
 *   4. 初始化硬件   —— 4.1 时基(仅首次) → 4.2 输出通道 → 4.3 GPIO
 *   5. 执行命令     —— 使能定时器和 PWM 输出
 *
 * 硬件绑定方式：
 *   通过比较指针 PWMx == PWM1 / PWM2 确定当前初始化的是哪个实例，
 *   然后从对应的 PWM1_TIMx / PWM2_TIMx 等宏读取硬件资源，
 *   填入全局池 PWM_BASE[N].hd 中。这种方式将硬件引脚映射集中
 *   在 .h 文件的宏定义中，修改引脚无需改动本函数。
 *
 * 频率换算：
 *   freq(Hz) → arr = SYSTEM_CLOCK_HZ / freq - 1
 *   例如 72MHz / 10000Hz - 1 = 7199 → PWM 频率 10kHz
 *   PSC 固定为 0，以获取最大分辨率。
 *===========================================================================*/
ErrorStatus PWM_Init(PWM_TypeDef_Struct *PWMx, PWM_InitTypeDef_Struct *PWM_InitStruct)
{
    // 1. 定义局部变量
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
    GPIO_InitTypeDef         GPIO_InitStructure;
    ErrorStatus              err = SUCCESS;

    // 2. 校验参数
    if((PWMx == NULL) || (PWM_InitStruct == NULL))
        return ERROR;

    // 3. 配置参数（硬件绑定 + 参数拷贝）
    if(PWMx == PWM1) {
        PWM_BASE[0].hd.PWM_TIMx      = PWM1_TIMx;
        PWM_BASE[0].hd.PWM_CHNx      = PWM1_CHNx;
        PWM_BASE[0].hd.PWM_GPIOx     = PWM1_GPIOx;
        PWM_BASE[0].hd.PWM_GPIO_Pinx = PWM1_GPIO_Pinx;
    } else if(PWMx == PWM2) {
        PWM_BASE[1].hd.PWM_TIMx      = PWM2_TIMx;
        PWM_BASE[1].hd.PWM_CHNx      = PWM2_CHNx;
        PWM_BASE[1].hd.PWM_GPIOx     = PWM2_GPIOx;
        PWM_BASE[1].hd.PWM_GPIO_Pinx = PWM2_GPIO_Pinx;
    } else {
        return ERROR;
    }

    PWMx->para.enable  = PWM_InitStruct->enable;
    PWMx->para.log     = PWM_InitStruct->log;
    /* freq → arr: TIM_CLK = 72MHz, arr = 72M / freq - 1 */
    PWMx->para.arr     = (uint16_t)((float)SYSTEM_CLOCK_HZ / PWM_InitStruct->freq) - 1;
    PWMx->para.psc     = 0;
    PWMx->para.dutyMax = PWM_InitStruct->dutyMax;
    PWMx->para.dutyMin = PWM_InitStruct->dutyMin;
    PWMx->para.duty    = PWM_InitStruct->duty;

    // 4. 初始化硬件
    if(err == SUCCESS) {
        // 4.1 初始化定时器时基（TIM3 仅需初始化一次）
        if(!pwm_tim3_inited) {
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

            TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
            TIM_TimeBaseInitStruct.TIM_Period        = PWMx->para.arr;
            TIM_TimeBaseInitStruct.TIM_Prescaler     = PWMx->para.psc;
            TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
            TIM_TimeBaseInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;
            TIM_TimeBaseInit(PWMx->hd.PWM_TIMx, &TIM_TimeBaseInitStruct);
            pwm_tim3_inited = 1;
        }

        // 4.2 初始化输出通道（每个通道独立配置）
        TIM_OCStructInit(&TIM_OCInitStructure);
        TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCPolarity  = (PWMx->para.log == PWM_LOG_POSITIVE)
                                              ? TIM_OCPolarity_High
                                              : TIM_OCPolarity_Low;
        TIM_OCInitStructure.TIM_Pulse       = 0;

        if(PWMx->hd.PWM_CHNx == TIM_Channel_1) {
            TIM_OC1Init(PWMx->hd.PWM_TIMx, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(PWMx->hd.PWM_TIMx, TIM_OCPreload_Enable);
        } else if(PWMx->hd.PWM_CHNx == TIM_Channel_2) {
            TIM_OC2Init(PWMx->hd.PWM_TIMx, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(PWMx->hd.PWM_TIMx, TIM_OCPreload_Enable);
        }

        // 4.3 初始化 GPIO（复用推挽输出，50MHz 驱动能力）
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin   = PWMx->hd.PWM_GPIO_Pinx;
        GPIO_Init(PWMx->hd.PWM_GPIOx, &GPIO_InitStructure);
    }

    // 5. 执行命令（使能定时器计数器 + 通道 PWM 输出）
    if((err == SUCCESS) && (PWMx->para.enable == ENABLE)) {
        TIM_Cmd(PWMx->hd.PWM_TIMx, ENABLE);
        TIM_CCxCmd(PWMx->hd.PWM_TIMx, PWMx->hd.PWM_CHNx, TIM_CCx_Enable);
    }

    return err;
}

/*===========================================================================
 * PWM_Enable —— 使能 PWM 输出
 *
 * 仅使能当前通道的 PWM 信号输出（CCx 输出到 GPIO）。
 * 注意：TIM3 计数器本身已在 Init 时启动，此处不重复操作。
 *===========================================================================*/
ErrorStatus PWM_Enable(PWM_TypeDef_Struct *PWMx)
{
    if(PWMx == NULL)
        return ERROR;

    PWMx->para.enable = ENABLE;
    TIM_CCxCmd(PWMx->hd.PWM_TIMx, PWMx->hd.PWM_CHNx, TIM_CCx_Enable);
    return SUCCESS;
}

/*===========================================================================
 * PWM_Disable —— 禁能 PWM 输出
 *
 * 关闭当前通道的 PWM 信号输出。GPIO 引脚变为无效电平（低电平），
 * 对应 TB6612 的 PWM=0，电机停止。
 * 与 Enable 成对使用，用于紧急停转或节能。
 *===========================================================================*/
ErrorStatus PWM_Disable(PWM_TypeDef_Struct *PWMx)
{
    if(PWMx == NULL)
        return ERROR;

    PWMx->para.enable = DISABLE;
    TIM_CCxCmd(PWMx->hd.PWM_TIMx, PWMx->hd.PWM_CHNx, TIM_CCx_Disable);
    return SUCCESS;
}

/*===========================================================================
 * PWM_setDuty —— 设置 PWM 占空比
 *
 * 实现原理：
 *   STM32 PWM1 模式下，当 CNT < CCR 时输出有效电平（高电平），
 *   当 CNT >= CCR 时输出无效电平（低电平）。
 *   因此 CCR = (ARR+1) × duty 可以直接控制占空比。
 *
 * 限幅保护：
 *   输入 duty 被钳位到 [dutyMin, dutyMax] 范围内，
 *   防止因上层计算错误导致的 0% 以下或 100% 以上输出。
 *
 * @param PWMx  PWM 实例指针（PWM1 或 PWM2）
 * @param duty  目标占空比 [0.0, 1.0]，超出范围自动限幅
 * @return      SUCCESS / ERROR
 *===========================================================================*/
ErrorStatus PWM_setDuty(PWM_TypeDef_Struct *PWMx, float duty)
{
    uint16_t compare;

    if(PWMx == NULL)
        return ERROR;

    /* 限幅到 [dutyMin, dutyMax] */
    if(duty > PWMx->para.dutyMax) duty = PWMx->para.dutyMax;
    if(duty < PWMx->para.dutyMin) duty = PWMx->para.dutyMin;

    PWMx->para.duty = duty;

    /* CCR = (ARR + 1) × duty
     * 例：ARR=7199, duty=0.5 → CCR=3600 → PWM 占空比 50% */
    compare = (uint16_t)((float)(PWMx->para.arr + 1) * duty);

    /* 根据通道号写对应比较寄存器 */
    if(PWMx->hd.PWM_CHNx == TIM_Channel_1) {
        TIM_SetCompare1(PWMx->hd.PWM_TIMx, compare);
    } else {
        TIM_SetCompare2(PWMx->hd.PWM_TIMx, compare);
    }

    return SUCCESS;
}
