/*******************************************************************************
 * @file    stm32f103_pwm.h
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   PWM 模块 —— 硬件抽象层
 *
 * 基于 TIM3 的双通道 PWM 输出模块，用于驱动 TB6612 电机驱动板的
 * 速度控制。两个通道共享 TIM3 定时器，各自独立控制占空比。
 *
 * 硬件资源：
 *   PWM1 → TIM3_CH1 (PA6)  左电机速度
 *   PWM2 → TIM3_CH2 (PA7)  右电机速度
 *
 * 设计模式：
 *   遵循 "硬件描述 + 运行参数 + 合并句柄" 三结构体模式。
 *   硬件资源绑定在 .h 文件中以宏定义集中管理，修改引脚只需改宏。
 *   全局实例池 PWM_BASE[N] 在 .c 中定义，宏别名 PWM1/PWM2 暴露给上层。
 *
 * 使用示例：
 *   PWM_InitTypeDef_Struct cfg;
 *   PWM_StructInit(&cfg);           // 填充默认值
 *   cfg.freq = 10000.0f;            // 可选：覆盖频率
 *   PWM_Init(PWM1, &cfg);           // 绑定硬件 + 配置 + 使能
 *   PWM_setDuty(PWM1, 0.5f);        // 设置 50% 占空比
 *******************************************************************************/

#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"
#include "bsp_config.h"

/*===========================================================================
 * 1. 默认值宏定义
 *
 * 命名格式：PWM_属性名_DEFAULT
 * 所有浮点数加 f 后缀，宏值加括号。
 *===========================================================================*/
#define PWM_ENABLE_DEFAULT      ENABLE              /* 上电默认使能 */
#define PWM_LOG_DEFAULT         PWM_LOG_POSITIVE    /* 默认极性：正逻辑（高有效） */
#define PWM_FREQ_DEFAULT        (10000.0f)          /* 默认PWM频率 10kHz */
#define PWM_DUTYMAX_DEFAULT     (1.0f)              /* 最大占空比 100% */
#define PWM_DUTYMIN_DEFAULT     (0.0f)              /* 最小占空比 0% */
#define PWM_DUTY_DEFAULT        (0.0f)              /* 初始占空比 0%（电机停止） */

/*===========================================================================
 * 2. 枚举定义
 *===========================================================================*/

/**
 * @brief PWM 输出极性
 *
 * PWM_LOG_POSITIVE: 高电平为有效电平（CCR 控制高电平时间）
 * PWM_LOG_NEGATIVE: 低电平为有效电平（CCR 控制低电平时间）
 * 配合 TB6612，使用正逻辑即可。
 */
typedef enum {
    PWM_LOG_POSITIVE = 0,
    PWM_LOG_NEGATIVE = 1
} PWM_log_Enum;

/*===========================================================================
 * 3. 结构体定义
 *
 * 四层结构体模式：
 *   _hd_Struct      → 硬件绑定，编译后不可变（定时器、通道、GPIO）
 *   _para_Struct    → 运行参数，运行时可变（使能、占空比、频率）
 *   _TypeDef_Struct → 合并句柄 = hd + para，作为模块的统一操作对象
 *   _InitTypeDef    → 用户初始化配置，仅暴露必要参数给调用者
 *===========================================================================*/

/**
 * @brief PWM 硬件描述结构体
 *
 * 记录本通道绑定的定时器、通道号、GPIO 端口和引脚。
 * 这些字段在 PWM_Init() 中根据宏别名（PWM1/PWM2）自动填充，
 * 用户无需手动设置。
 */
typedef struct {
    TIM_TypeDef*   PWM_TIMx;       /* 定时器基地址，如 TIM3 */
    uint16_t       PWM_CHNx;       /* 通道号，如 TIM_Channel_1 */
    GPIO_TypeDef*  PWM_GPIOx;      /* GPIO 端口，如 GPIOA */
    uint16_t       PWM_GPIO_Pinx;  /* GPIO 引脚，如 GPIO_Pin_6 */
} PWM_hd_Struct;

/**
 * @brief PWM 运行参数结构体
 *
 * 保存 PWM 运行时的所有可变状态。
 * duty / dutyMax / dutyMin 用于限幅保护，防止占空比超出硬件能力。
 * arr / psc 由 freq 自动换算：arr = TIM_CLK / freq - 1, psc = 0。
 */
typedef struct {
    FunctionalState  enable;       /* ENABLE=输出使能, DISABLE=输出禁止 */
    PWM_log_Enum     log;          /* 输出极性（正逻辑/负逻辑） */
    uint16_t         arr;          /* 自动重装载值（决定 PWM 周期） */
    uint16_t         psc;          /* 预分频值 */
    float            dutyMax;      /* 占空比上限 [0.0, 1.0] */
    float            dutyMin;      /* 占空比下限 [0.0, 1.0] */
    float            duty;         /* 当前占空比 [dutyMin, dutyMax] */
} PWM_para_Struct;

/**
 * @brief PWM 合并句柄（硬件 + 参数）
 *
 * 所有 PWM API 的第一个参数均为指向此结构体的指针。
 * 外部通过宏别名 PWM1 / PWM2 获取实例指针。
 */
typedef struct {
    PWM_hd_Struct    hd;           /* 硬件绑定（不可变） */
    PWM_para_Struct  para;         /* 运行参数（可变） */
} PWM_TypeDef_Struct;

/**
 * @brief PWM 初始化配置结构体
 *
 * 仅暴露给用户的可配置项。未在此结构体中的参数（如 PSC）由 Init 内部处理。
 * 使用流程：先调用 PWM_StructInit() 填默认值，再按需覆盖个别字段，
 * 最后传入 PWM_Init()。
 */
typedef struct {
    FunctionalState  enable;       /* 初始化后是否立即使能 */
    PWM_log_Enum     log;          /* 输出极性 */
    float            freq;         /* PWM 频率（Hz），Init 自动换算 arr/psc */
    float            dutyMax;      /* 占空比上限 */
    float            dutyMin;      /* 占空比下限 */
    float            duty;         /* 初始占空比 */
} PWM_InitTypeDef_Struct;

/*===========================================================================
 * 4. 全局实例池 + 宏别名
 *
 * 每个实例的硬件资源用 MODULE_N_PROPERTY 命名：
 *   PWM1_TIMx, PWM1_CHNx, PWM1_GPIOx, PWM1_GPIO_Pinx
 * 实例指针用 MODULEN 格式：
 *   PWM1 → (&PWM_BASE[0]), PWM2 → (&PWM_BASE[1])
 *
 * 修改引脚/定时器只需改下面的宏，无需改动 .c 文件。
 *===========================================================================*/
#define PWM_BASE_NUM        2
extern PWM_TypeDef_Struct PWM_BASE[PWM_BASE_NUM];

/* PWM1: 左电机 PWM → TIM3_CH1 (PA6)
 * 原理图连接：STM32 PA6 → TB6612 PWMA（左电机速度控制） */
#define PWM1                (&PWM_BASE[0])
#define PWM1_TIMx           TIM3
#define PWM1_CHNx           TIM_Channel_1
#define PWM1_GPIOx          GPIOA
#define PWM1_GPIO_Pinx      GPIO_Pin_6

/* PWM2: 右电机 PWM → TIM3_CH2 (PA7)
 * 原理图连接：STM32 PA7 → TB6612 PWMB（右电机速度控制） */
#define PWM2                (&PWM_BASE[1])
#define PWM2_TIMx           TIM3
#define PWM2_CHNx           TIM_Channel_2
#define PWM2_GPIOx          GPIOA
#define PWM2_GPIO_Pinx      GPIO_Pin_7

/*===========================================================================
 * 5. API 函数声明
 *
 * 统一签名规范：
 *   - 返回值一律 ErrorStatus（SUCCESS/ERROR），不返回 void
 *   - 第一个参数永远是指向自身句柄的指针（PWM_TypeDef_Struct*）
 *   - 函数体第一行做 NULL 检查
 *===========================================================================*/

ErrorStatus PWM_StructInit(PWM_InitTypeDef_Struct *p);
ErrorStatus PWM_Init(PWM_TypeDef_Struct *PWMx, PWM_InitTypeDef_Struct *PWM_InitStruct);
ErrorStatus PWM_Enable(PWM_TypeDef_Struct *PWMx);
ErrorStatus PWM_Disable(PWM_TypeDef_Struct *PWMx);
ErrorStatus PWM_setDuty(PWM_TypeDef_Struct *PWMx, float duty);

#endif /* __PWM_H */
