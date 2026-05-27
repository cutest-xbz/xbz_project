/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.

*/
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_




/* 循环任务指针结构体 */
typedef struct __SCHEDULER_TASKS_Struct
{
		void (*pDuty_1ms)(void);    /*1ms周期任务的指针变量*/
		void (*pDuty_2ms)(void);    /*2ms周期任务的指针变量*/
		void (*pDuty_3ms)(void);    /*2ms周期任务的指针变量*/
		void (*pDuty_4ms)(void);    /*2ms周期任务的指针变量*/	
		void (*pDuty_5ms)(void);    /*5ms周期任务的指针变量*/
		void (*pDuty_10ms)(void);   /*10ms周期任务的指针变量*/
		void (*pDuty_20ms)(void);   /*20ms周期任务的指针变量*/
		void (*pDuty_50ms)(void);   /*50ms周期任务的指针变量*/
		void (*pDuty_100ms)(void);  /*100ms周期任务的指针变量*/
} SCHEDULER_TASKS_Struct;


/*----------------------------------------------------------
 + 实现功能：调度机，初始化
----------------------------------------------------------*/
void Scheduler_Init(SCHEDULER_TASKS_Struct initTask);

/*----------------------------------------------------------
 + 实现功能：调度机，启动
----------------------------------------------------------*/
void Scheduler_Start(void);

/*----------------------------------------------------------
 + 实现功能：调度机，执行循环
----------------------------------------------------------*/
void Scheduler_Main_Loop(void);
#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
