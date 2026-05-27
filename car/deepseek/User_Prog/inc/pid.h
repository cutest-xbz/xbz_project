/**
 * @file    pid.h
 * @brief   PID 控制器模块 —— 头文件
 *
 * 提供两种 PID 算法：
 *   1. 增量式（推荐用于电机速度环）：
 *      Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
 *      u(k)  = u(k-1) + Δu(k)
 *      优点：天然抗积分饱和，输出平滑
 *
 *   2. 位置式：
 *      u(k)  = Kp*e(k) + Ki*Σe(k) + Kd*(e(k)-e(k-1))
 */

#ifndef __PID_H
#define __PID_H

#include "bsp_config.h"

typedef enum {
    PID_MODE_POSITION  = 0,
    PID_MODE_INCREMENT = 1
} PID_Mode_t;

typedef struct {
    PID_Mode_t  mode;           /* 工作模式 */
    float       kp, ki, kd;     /* PID 系数 */
    float       output_max;     /* 输出上限 */
    float       output_min;     /* 输出下限 */

    /* 内部状态 */
    float       target;
    float       last_error;
    float       prev_error;
    float       integral;
    float       last_output;
    uint8_t     first_run;
} PID_t;

void  PID_Init(PID_t *pid, PID_Mode_t mode,
               float kp, float ki, float kd,
               float output_max, float output_min);
void  PID_SetTarget(PID_t *pid, float target);
void  PID_Tune(PID_t *pid, float kp, float ki, float kd);
float PID_Compute(PID_t *pid, float target, float actual);
void  PID_Reset(PID_t *pid);

#endif /* __PID_H */
