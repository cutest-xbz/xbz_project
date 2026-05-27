/*******************************************************************************
 * @file    stm32f103_tracker.h
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   灰度传感器模块 —— 硬件抽象层
 *
 * 8 路红外数字型循迹传感器驱动，用于智能小车黑线检测与位置偏差计算。
 *
 * 传感器原理：
 *   每个传感器为红外对管：发射管发出红外光，接收管检测地面反射。
 *   - 黑线吸收红外光 → 反射弱 → 模块输出低电平 (0)
 *   - 白色地面反射强 → 模块输出高电平 (1)
 *   8 个传感器一字排列，通过加权求和算法计算出黑线的连续位置偏差。
 *
 * 硬件资源：
 *   TRACKER1 → GPIOD PD0~PD7（8路传感器，从左到右排列）
 *
 * 设计要点：
 *   - STM32F103ZE (LQFP144) 的 OSC_IN/OSC_OUT 在 PH0/PH1，PD0/PD1 为独立 GPIO，
 *     因此可以同时使用 HSE 外部晶振和 GPIOD 全 16 路传感器。
 *   - 8 通道共享同一 GPIO 端口，Init 中一次性配置所有引脚。
 *   - 权重数组 TRACKER1_CH_WEIGHTS 左负右正，用于连续位置计算。
 *
 * 使用示例：
 *   TRACKER_InitTypeDef_Struct cfg;
 *   TRACKER_StructInit(&cfg);
 *   TRACKER_Init(TRACKER1, &cfg);
 *   // 每 10ms:
 *   float error = TRACKER_CalcError(TRACKER1);  // [-1.0, +1.0]
 *   uint8_t ok = TRACKER_IsLineDetected(TRACKER1);
 *******************************************************************************/

#ifndef __TRACKER_H
#define __TRACKER_H

#include "stm32f10x.h"
#include "bsp_config.h"

/*===========================================================================
 * 1. 默认值宏定义
 *===========================================================================*/
#define TRACKER_ENABLE_DEFAULT       ENABLE    /* 上电默认使能 */
#define TRACKER_CHANNEL_NUM          8         /* 传感器路数（硬件固定） */
#define TRACKER_WEIGHT_MAX           7      /* 最大权重绝对值（归一化分母） */

/*===========================================================================
 * 2. 结构体定义
 *
 * 与 PWM/Motor 不同，Tracker 的 8 个通道共用一块硬件（同一 GPIO 端口），
 * 因此 hd 中包含 8 个通道的数组，para 包含模块级的运行状态。
 *===========================================================================*/

/**
 * @brief 单通道硬件描述 —— 绑定一个传感器引脚
 *
 * 每个通道独立绑定一个 GPIO 引脚，8 个通道全部在同一 GPIO 端口上（GPIOD）。
 */
typedef struct {
    GPIO_TypeDef*  GPIOx;          /* 所属 GPIO 端口，如 GPIOD */
    uint16_t       GPIO_Pinx;      /* 引脚号，如 GPIO_Pin_0 */
} TRACKER_Ch_hd_Struct;

/**
 * @brief 单通道运行参数
 *
 * state: 传感器原始读数（0=检测到黑线, 1=白色地面）
 * weight: 该传感器的位置权重，左侧为负、右侧为正
 * active: 该传感器当前是否在黑线上（1=是，由 CalcError 每周期更新）
 */
typedef struct {
    uint8_t  state;
    int8_t   weight;
    uint8_t  active;
} TRACKER_Ch_para_Struct;

/**
 * @brief 模块硬件描述 —— 8 通道引脚绑定
 */
typedef struct {
    TRACKER_Ch_hd_Struct    ch[TRACKER_CHANNEL_NUM];
} TRACKER_hd_Struct;

/**
 * @brief 模块运行参数
 *
 * lineDetected: 上一周期是否检测到黑线（供丢线处理策略使用）
 * lastValidError: 最近一次有效误差值，丢线时维持此值防止小车失控
 */
typedef struct {
    FunctionalState  enable;
    uint8_t          lineDetected;
    float            lastValidError;
} TRACKER_para_Struct;

/**
 * @brief Tracker 合并句柄
 *
 * hd:        8 通道的硬件绑定
 * para:      模块级运行状态
 * ch_para:   8 通道各自的状态缓存（权重在 Init 时绑定，state/active 每周期更新）
 */
typedef struct {
    TRACKER_hd_Struct        hd;
    TRACKER_para_Struct      para;
    TRACKER_Ch_para_Struct   ch_para[TRACKER_CHANNEL_NUM];
} TRACKER_TypeDef_Struct;

/**
 * @brief Tracker 初始化配置结构体
 */
typedef struct {
    FunctionalState  enable;
} TRACKER_InitTypeDef_Struct;

/*===========================================================================
 * 3. 全局实例池 + 宏别名
 *
 * TRACKER1 将 8 路传感器作为一个整体对外暴露。
 * 通道引脚和权重数组在 Init 时从宏展开赋值。
 *===========================================================================*/
#define TRACKER_BASE_NUM        1
extern TRACKER_TypeDef_Struct TRACKER_BASE[TRACKER_BASE_NUM];

/* TRACKER1: 8路红外传感器 → GPIOD PD0~PD7，左到右排列 */
#define TRACKER1                     (&TRACKER_BASE[0])
#define TRACKER1_GPIOx               GPIOD
#define TRACKER1_RCC_CLK             RCC_APB2Periph_GPIOD
/* 8 路引脚：PD0(最左) ~ PD7(最右) */
#define TRACKER1_CH_PINS             {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, \
                                      GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7}
/* 位置权重：左负右正，[-7,-5,-3,-1,1,3,5,7]
 * 例如 S4(权重-1) + S5(权重+1) 同时激活 → 误差 = (-1+1)/2/7 = 0.0（居中） */
#define TRACKER1_CH_WEIGHTS          {7, 5, 3, 1, -1, -3, -5, -7}

/*===========================================================================
 * 4. API 函数声明
 *===========================================================================*/
ErrorStatus TRACKER_StructInit(TRACKER_InitTypeDef_Struct *p);
ErrorStatus TRACKER_Init(TRACKER_TypeDef_Struct *TRACKERx, TRACKER_InitTypeDef_Struct *TRACKER_InitStruct);
ErrorStatus TRACKER_Enable(TRACKER_TypeDef_Struct *TRACKERx);
ErrorStatus TRACKER_Disable(TRACKER_TypeDef_Struct *TRACKERx);
float      TRACKER_CalcError(TRACKER_TypeDef_Struct *TRACKERx);
uint8_t    TRACKER_IsLineDetected(TRACKER_TypeDef_Struct *TRACKERx);

#endif /* __TRACKER_H */
