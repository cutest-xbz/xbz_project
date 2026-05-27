# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

STM32F103ZE 智能循迹小车。单闭环位置 PID 控制，8 路红外传感器 + TB6612 电机驱动，Keil MDK + SPL v3.5 开发。

## Build & Flash

- IDE: MDK-ARM v5 (Keil)，工程文件 `project/model.uvprojx`
- 编译输出: `out/` 目录
- 烧录: J-Link (`project/JLinkSettings.ini`)
- 编译器: Arm Compiler 5/6 (取决于 MDK 版本)
- 无命令行构建脚本，需通过 MDK IDE 或 `uvprojx` 命令行编译

## Architecture: 4-Layer Strict Hierarchy

铁律：上层只调紧邻下层，绝不跨层。

```
Layer 3  User_App/main.c       系统初始化 + 调度器主循环，只调 CarDrive API
Layer 2  User_Prog/car_drive.c  差速转向控制，注入 Motor×2 + Tracker + PID
         User_Prog/pid.c        位置式/增量式 PID 算法
Layer 1  User_Driver/motor.c    TB6612 方向 + 编码器(16→32位扩展, 低通滤波)
         User_Driver/pwm.c      TIM3 CH1/CH2 10kHz PWM
         User_Driver/tracker.c  8路红外加权求和连续位置估计
Layer 0  FWlib/                 STM32 SPL v3.5 标准外设库
System   system/scheduler.c     SysTick 1ms 驱动任务调度
         system/time.c          SysTick 钩子 + 延时 + 计时
```

## Hardware & Pin Map

| 功能 | 引脚 | 定时器 |
|------|------|--------|
| 左电机 PWM | PA6 | TIM3_CH1, 10kHz |
| 右电机 PWM | PA7 | TIM3_CH2, 10kHz |
| 左电机 IN1/IN2 | PD9/PD11 | TB6612 |
| 右电机 IN1/IN2 | PD12/PD10 | TB6612 |
| 左编码器 A/B | PA0/PA1 | TIM2 (已配,未用) |
| 右编码器 A/B | PB6/PB7 | TIM4 (已配,未用) |
| 红外 S1~S8 | PD0~PD7 | GPIOD 上拉输入 |

主频 72MHz (HSE 8MHz → PLL ×9)，主控 STM32F103ZE (LQFP144)。小车为四轮 2WD 差速驱动，无编码器反馈。

## Key Design Patterns

**依赖注入 (IoC)**：上层在 Init 前设置下层句柄指针，不直接访问下层全局变量。
```c
MOTOR1->hd.PWMx = PWM1;           // Motor 注入 PWM 句柄
CAR1->hd.motor_L = MOTOR1;        // CarDrive 注入 Motor 句柄
CAR1->hd.tracker = TRACKER1;      // CarDrive 注入 Tracker 句柄
```

**三结构体模式**：`hd`(硬件绑定, 编译时确定) + `para`(运行参数, 运行时可变) → `TypeDef`(合并句柄)。每个模块的 Init 结构体和类型定义遵循此模式。

**全局实例池 + 宏别名**：实例存于 `MODULE_BASE[N]` 数组，通过 `MODULEN` 宏访问（如 `PWM1 = &PWM_BASE[0]`）。

**五步 Init**：声明局部变量 → 校验参数 → 配置参数(硬件绑定) → 初始化硬件 → 执行命令。所有模块的 `_Init()` 函数遵循此模板。

**Signed Speed 接口**：`MOTOR_setSpeed(float speed)`，符号表方向（正=正转, 负=反转, 0=刹车），绝对值表速度。刹车时 IN1=IN2=H (H桥短路制动)。

## Control Strategy

- 周期：10ms (SysTick 1ms → 调度器 → `task_10ms()` → `CARDRIVE_run(CAR1)`)
- 位置式 PID：`u(k) = Kp×e(k) + Kd×(e(k)-e(k-1))`，Ki=0
- 差速转向：`left=basePWM+turn, right=basePWM-turn`，限幅 [0, speedMax]
- 默认参数：Kp=3.0, Ki=0, Kd=1.0, basePWM=0.4, 输出限幅 ±0.5
- 丢线策略：保持 lastValidError 惯性直行；紧急停车调用 `CARDRIVE_EmergencyStop()`
- 运行时调参：`CARDRIVE_tunePID(CAR1, kp, ki, kd)`, `CARDRIVE_setBasePWM(CAR1, pwm)`

## Tracker Algorithm

8 路传感器权重 `{-7,-5,-3,-1,1,3,5,7}`，左负右正。加权质心法计算黑线连续位置：
```
error = Σ(weight[i] × active[i]) / active_count / 7
```
归一化到 [-1.0, +1.0]。`TRACKER_WEIGHT_MAX=7` 必须等于权重数组最大绝对值。权重值必须与传感器到中心的实际物理距离成比例（非等间距布局需重新标定）。

## Constraints

- 无编码器反馈，仅单闭环（位置环），速度环为开环 PWM 直接输出
- 不支持命令行编译，所有构建通过 Keil MDK IDE
- 代码为裸机运行，调度器非抢占式——所有任务必须在下一周期到来前完成，否则 `err_flag` 计数
- 浮点运算开销需关注（软件 FPU, Cortex-M3 无硬件浮点）
