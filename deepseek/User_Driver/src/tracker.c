/*******************************************************************************
 * @file    stm32f103_tracker.c
 * @author  SmartCar Team
 * @version V1.0.0
 * @date    06/05/2026
 * @brief   灰度传感器模块实现：8路红外循迹，加权求和位置计算
 *
 * 硬件细节：
 *   - 8 路传感器全部连接到 GPIOD PD0~PD7
 *   - 引脚配置为上拉输入（IPU）：默认高电平=白色，检测到黑线拉低
 *   - 所有引脚在 Init 中一次性配置，无需逐个操作
 *
 * 算法核心：加权求和法（Weighted Sum）
 *   将 8 个离散的数字传感器读数合成为一个连续的 [-1.0, +1.0] 位置值，
 *   实现"亚像素级"的线位置估计，远超 8 级离散分辨率。
 *
 * 丢线处理策略：
 *   当所有传感器都不在黑线上时（小车冲出赛道），保持上一次有效误差。
 *   这等效于"惯性直行"，给小车机会自行找回黑线。
 *   如需紧急停车，上层 CarDrive 可调用 CARDRIVE_EmergencyStop()。
 *******************************************************************************/

#include "tracker.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stddef.h>

/*===========================================================================
 * 全局实例池
 *===========================================================================*/
TRACKER_TypeDef_Struct TRACKER_BASE[TRACKER_BASE_NUM];

/*===========================================================================
 * TRACKER_StructInit —— 用默认值填充 Init 结构体
 *
 * 当前仅 enable 一个可配置项。保留此框架以便后续扩展
 * （如增加触发阈值、采样次数等参数）。
 *===========================================================================*/
ErrorStatus TRACKER_StructInit(TRACKER_InitTypeDef_Struct *p)
{
    if(p == NULL)
        return ERROR;

    p->enable = TRACKER_ENABLE_DEFAULT;

    return SUCCESS;
}

/*===========================================================================
 * TRACKER_Init —— 初始化 8 路灰度传感器（五步法）
 *
 * 硬件绑定方式：
 *   通过 TRACKERx == TRACKER1 确认实例合法，然后从 TRACKER1_CH_PINS
 *   和 TRACKER1_CH_WEIGHTS 宏展开数组，逐通道填入 ch_para[i].weight
 *   和 hd.ch[i] 的引脚绑定。初始状态全部设为"白色地面"（state=1）。
 *
 * GPIO 配置要点：
 *   所有 8 路引脚配置为上拉输入（GPIO_Mode_IPU）。
 *   上拉确保传感器未触发时读到高电平（白色），触发时传感器拉低（黑线）。
 *   8 个引脚的 GPIOD 时钟在 main.c RCC_Configuration 中已有全局使能，
 *   此处重复使能是防御性编程，不会出错。
 *===========================================================================*/
ErrorStatus TRACKER_Init(TRACKER_TypeDef_Struct *TRACKERx, TRACKER_InitTypeDef_Struct *TRACKER_InitStruct)
{
    // 1. 定义局部变量
    GPIO_InitTypeDef  GPIO_InitStructure;
    ErrorStatus       err = SUCCESS;
    uint8_t           i;
    const uint16_t    default_pins[TRACKER_CHANNEL_NUM] = TRACKER1_CH_PINS;
    const int8_t      default_weights[TRACKER_CHANNEL_NUM] = TRACKER1_CH_WEIGHTS;

    // 2. 校验参数
    if((TRACKERx == NULL) || (TRACKER_InitStruct == NULL))
        return ERROR;

    // 3. 配置参数（硬件绑定 + 参数拷贝）
    if(TRACKERx == TRACKER1) {
        for(i = 0; i < TRACKER_CHANNEL_NUM; i++) {
            TRACKER_BASE[0].hd.ch[i].GPIOx     = TRACKER1_GPIOx;
            TRACKER_BASE[0].hd.ch[i].GPIO_Pinx = default_pins[i];
            TRACKER_BASE[0].ch_para[i].weight  = default_weights[i];
            TRACKER_BASE[0].ch_para[i].state   = 1;     /* 初始=白色 */
            TRACKER_BASE[0].ch_para[i].active  = 0;     /* 未激活 */
        }
    } else {
        return ERROR;
    }

    TRACKERx->para.enable         = TRACKER_InitStruct->enable;
    TRACKERx->para.lastValidError = 0.0f;
    TRACKERx->para.lineDetected   = 0;

    // 4. 初始化硬件
    if(err == SUCCESS) {
        // 4.1 使能 GPIO 时钟（幂等操作，可重复调用）
        RCC_APB2PeriphClockCmd(TRACKER1_RCC_CLK, ENABLE);

        // 4.2 配置所有 8 路传感器引脚为上拉输入
        //     一次性配置 PD0~PD7，避免循环调用 GPIO_Init 8 次
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                                      | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_Init(TRACKERx->hd.ch[0].GPIOx, &GPIO_InitStructure);
    }

    // 5. 执行命令（传感器为被动器件，无需启动信号）
    return err;
}

/*===========================================================================
 * TRACKER_Enable —— 使能传感器模块
 *
 * 软件使能标记。传感器本身为无源器件，无需硬件操作。
 * 主要用于配合调度器：禁能时 CalcError 仍可调用但返回值无效。
 *===========================================================================*/
ErrorStatus TRACKER_Enable(TRACKER_TypeDef_Struct *TRACKERx)
{
    if(TRACKERx == NULL)
        return ERROR;

    TRACKERx->para.enable = ENABLE;
    return SUCCESS;
}

/*===========================================================================
 * TRACKER_Disable —— 禁能传感器模块
 *
 * 与 Enable 成对。禁能后上层应停止读取避免使用过期数据。
 *===========================================================================*/
ErrorStatus TRACKER_Disable(TRACKER_TypeDef_Struct *TRACKERx)
{
    if(TRACKERx == NULL)
        return ERROR;

    TRACKERx->para.enable = DISABLE;
    return SUCCESS;
}

/*===========================================================================
 * TRACKER_CalcError —— 加权求和法计算连续位置误差（核心算法）
 *
 * ── 为什么不用简单的"哪个传感器看到了黑线"？ ──
 * 8 个离散传感器只能区分 8 个位置，且相邻传感器之间是"盲区"。
 * 加权求和法利用"黑线有一定宽度、会同时触发多个传感器"的事实，
 * 通过求激活传感器的加权质心，得到连续的亚像素级位置估计。
 *
 * ── 算法步骤（三步法）──
 *
 * 第一步：读取所有传感器原始状态
 *   遍历 GPIO 读取 8 个引脚电平：
 *     Bit_RESET(0) → 低电平 → 检测到黑线 → state=0, active=1
 *     Bit_SET(1)    → 高电平 → 白色地面   → state=1, active=0
 *
 * 第二步：累加所有检测到黑线的传感器权重
 *   weighted_sum = Σ(weight[i] × active[i])
 *   active_count = Σ(active[i])
 *
 * 第三步：计算归一化误差
 *   error = weighted_sum / active_count / TRACKER_WEIGHT_MAX
 *
 * ── 公式推导 ──
 * 设左到右 8 个传感器，权重为 {-7,-5,-3,-1,1,3,5,7}。
 *
 * 示例 1：只有 S4（权重-1）检测到
 *   error = (-1) / 1 / 7 = -0.143（略偏左）
 *
 * 示例 2：S4（权重-1）和 S5（权重+1）同时检测
 *   error = (-1+1) / 2 / 7 = 0.0（完美居中）
 *
 * 示例 3：只有 S8（权重+7）检测到
 *   error = 7 / 1 / 7 = 1.0（严重偏右）
 *
 * 示例 4：S1~S3（权重-7,-5,-3）同时检测
 *   error = (-7-5-3) / 3 / 7 = -0.714（明显偏左，更平滑的估计）
 *
 * ── 丢线处理 ──
 * active_count == 0 时（全部白色）：
 *   设置 lineDetected = 0，返回 lastValidError。
 *   小车将保持上次有效误差对应的方向继续前进，
 *   直到重新检测到黑线。这比立即停车更平滑。
 *===========================================================================*/
float TRACKER_CalcError(TRACKER_TypeDef_Struct *TRACKERx)
{
    int16_t  weighted_sum = 0;
    uint8_t  active_count = 0;
    uint8_t  i;

    if(TRACKERx == NULL)
        return 0.0f;

    /* 第一步：读取所有传感器原始状态
     * Bit_RESET = 检测到黑线，Bit_SET = 白色地面 */
    for(i = 0; i < TRACKER_CHANNEL_NUM; i++) {
        if(GPIO_ReadInputDataBit(TRACKERx->hd.ch[i].GPIOx,
                                  TRACKERx->hd.ch[i].GPIO_Pinx) == Bit_RESET) {
            TRACKERx->ch_para[i].state  = 0;   /* 低电平 → 黑线 */
            TRACKERx->ch_para[i].active = 1;
        } else {
            TRACKERx->ch_para[i].state  = 1;   /* 高电平 → 白色 */
            TRACKERx->ch_para[i].active = 0;
        }
    }

    /* 第二步：累加所有检测到黑线的传感器权重 */
    for(i = 0; i < TRACKER_CHANNEL_NUM; i++) {
        if(TRACKERx->ch_para[i].active) {
            weighted_sum += TRACKERx->ch_para[i].weight;
            active_count++;
        }
    }

    /* 第三步：计算归一化误差
     * 多传感器时：error = 加权平均 / max_weight → 连续值
     * 无传感器时：保持上次有效误差（惯性直行策略） */
    if(active_count > 0) {
        float error = (float)weighted_sum / (float)active_count
                     / (float)TRACKER_WEIGHT_MAX;
        TRACKERx->para.lastValidError = error;
        TRACKERx->para.lineDetected   = 1;
        return error;
    } else {
        TRACKERx->para.lineDetected = 0;
        return TRACKERx->para.lastValidError;
    }
}

/*===========================================================================
 * TRACKER_IsLineDetected —— 查询当前是否检测到黑线
 *
 * 返回值在每次 TRACKER_CalcError() 调用后更新。
 * 上层可通过此函数实现丢线处理（如减速、停车、回转搜索）。
 *
 * @return 1=有黑线, 0=全部白色（丢线）
 *===========================================================================*/
uint8_t TRACKER_IsLineDetected(TRACKER_TypeDef_Struct *TRACKERx)
{
    if(TRACKERx == NULL)
        return 0;

    return TRACKERx->para.lineDetected;
}
