/* ©2015-2016 hznu. All rights reserved.

*/

#include "stm32f10x.h"
#include "device_timer.h"

/*----------------------------------------------------------
 + 实现功能：开启定时器
----------------------------------------------------------*/
void SysTick_Configuration(void)
{
    RCC_ClocksTypeDef  rcc_clocks;
    uint32_t         cnts;

    RCC_GetClocksFreq(&rcc_clocks);
    cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
    cnts = cnts / 8;

    /* 先设时钟源，再启动 SysTick */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    SysTick_Config(cnts);
}

/* ©2015-2016 hznu. All rights reserved. */
