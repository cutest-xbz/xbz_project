/**
 * @file    pid.c
 * @brief   PID 控制器模块 —— 实现文件
 *
 * 两种 PID 算法实现：
 *
 * 1. 位置式 PID：
 *    u(k) = Kp*e(k) + Ki*Σe(k) + Kd*(e(k) - e(k-1))
 *    直接输出绝对控制量，需额外做积分限幅防止饱和。
 *
 * 2. 增量式 PID（推荐用于电机速度控制）：
 *    Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
 *    u(k)  = u(k-1) + Δu(k)
 *    天然抗积分饱和，输出平滑，MCU 运算量小。
 *
 * PID 调参经验：
 *   1. Ki=0, Kd=0，逐步增大 Kp 至系统出现轻微震荡
 *   2. Kp 回退到震荡值的 60%~70%
 *   3. 逐步增大 Ki 消除稳态误差
 *   4. 加入 Kd 抑制过冲和震荡
 */

#include "pid.h"
#include <string.h>
#include <stddef.h>

/*===========================================================================
 * 初始化
 *===========================================================================*/

/**
 * @brief 初始化 PID 控制器
 * @param pid         PID 实例指针
 * @param mode        模式（位置式 / 增量式）
 * @param kp, ki, kd  PID 参数
 * @param output_max  输出上限
 * @param output_min  输出下限
 */
void PID_Init(PID_t *pid, PID_Mode_t mode,
              float kp, float ki, float kd,
              float output_max, float output_min)
{
    if (pid == NULL) return;

    memset(pid, 0, sizeof(PID_t));

    pid->mode       = mode;
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->first_run  = 1;
}

/*===========================================================================
 * 运行时参数调整
 *===========================================================================*/

/**
 * @brief 设置 PID 目标值
 */
void PID_SetTarget(PID_t *pid, float target)
{
    if (pid == NULL) return;
    pid->target = target;
}

/**
 * @brief 在线调整 PID 参数（无需重新初始化）
 *
 * 【注意】大幅改变 Ki 后建议调用 PID_Reset() 清除历史积分，
 *         避免因积分累积导致瞬间冲击。
 */
void PID_Tune(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/*===========================================================================
 * PID 核心计算
 *===========================================================================*/

/**
 * @brief PID 计算（每个控制周期调用一次）
 *
 * @param pid      PID 实例指针
 * @param target   本次目标值
 * @param actual   本次实际值（传感器/编码器反馈）
 * @return         控制输出量（已限幅到 [output_min, output_max]）
 *
 * 对于增量式 PID：
 *   - 输出是基于上一次输出的增量累加值
 *   - 首次调用返回 0（初始化误差历史）
 * 对于位置式 PID：
 *   - 输出是绝对控制量
 *   - 积分有防饱和限幅
 */
float PID_Compute(PID_t *pid, float target, float actual)
{
    float error_now;
    float output = 0.0f;

    if (pid == NULL) return 0.0f;

    pid->target = target;
    error_now   = target - actual;

    /* 首次运行：初始化误差历史，返回 0 */
    if (pid->first_run) {
        pid->last_error  = error_now;
        pid->prev_error  = error_now;
        pid->integral    = 0.0f;
        pid->last_output = 0.0f;
        pid->first_run   = 0;
        return 0.0f;
    }

    if (pid->mode == PID_MODE_POSITION) {
        float p_term, i_term, d_term;
        /* ================================================================
         * 位置式 PID: u(k) = Kp*e(k) + Ki*Σe(k) + Kd*(e(k)-e(k-1))
         * ================================================================ */
        p_term = pid->kp * error_now;

        /* 积分累加并限幅（防止深度积分饱和） */
        pid->integral += error_now;
        if (pid->integral > pid->output_max / (pid->ki + 0.001f)) {
            pid->integral = pid->output_max / (pid->ki + 0.001f);
        } else if (pid->integral < pid->output_min / (pid->ki + 0.001f)) {
            pid->integral = pid->output_min / (pid->ki + 0.001f);
        }
        i_term = pid->ki * pid->integral;

        d_term = pid->kd * (error_now - pid->last_error);

        output = p_term + i_term + d_term;

    } else {
        float p_delta, i_delta, d_delta, delta_u;
        /* ================================================================
         * 增量式 PID:
         *   Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
         *   u(k) = u(k-1) + Δu
         * ================================================================ */
        p_delta = pid->kp * (error_now - pid->last_error);
        i_delta = pid->ki * error_now;
        d_delta = pid->kd * (error_now - 2.0f * pid->last_error + pid->prev_error);

        delta_u = p_delta + i_delta + d_delta;

        output = pid->last_output + delta_u;
    }

    /* ---- 输出限幅（anti-windup） ---- */
    if (output > pid->output_max) {
        output = pid->output_max;
        if (pid->mode == PID_MODE_INCREMENT) {
            pid->last_output = pid->output_max;
        }
    } else if (output < pid->output_min) {
        output = pid->output_min;
        if (pid->mode == PID_MODE_INCREMENT) {
            pid->last_output = pid->output_min;
        }
    } else {
        pid->last_output = output;
    }

    /* ---- 更新误差历史 ---- */
    pid->prev_error = pid->last_error;
    pid->last_error = error_now;

    return output;
}

/*===========================================================================
 * 状态重置
 *===========================================================================*/

/**
 * @brief 重置 PID 内部状态
 *
 * 以下情况需调用此函数：
 *   - 从停止切换到运行
 *   - 控制目标大幅变化
 *   - 电机方向反转
 *   - 在线调参后积分过多
 */
void PID_Reset(PID_t *pid)
{
    if (pid == NULL) return;
    pid->last_error  = 0.0f;
    pid->prev_error  = 0.0f;
    pid->integral    = 0.0f;
    pid->last_output = 0.0f;
    pid->first_run   = 1;
}
