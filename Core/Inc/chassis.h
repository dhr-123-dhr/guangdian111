#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "motor.h"
#include "trap_plan.h"

/* ================================================================
 *  机 械 参 数 — 请根据实际小车填入
 * ================================================================
 */
#define WHEEL_DIAMETER_MM           85.0f      /* 轮子直径 (mm) */
#define WHEEL_BASE_MM               166.0f     /* 两轮间距/轮距 (mm) */
#define STEPS_PER_REV               3200       /* 每转脉冲数 (200步 × 16细分) */

/* ================================================================
 *  运 动 参 数
 * ================================================================
 */
#define MOTOR_MAX_SPEED_RPM         100.0f     /* 最高转速 (RPM) */
#define MOTOR_MIN_SPEED_RPM         5.0f       /* 起步最低转速 (RPM), 跳过电机低速共振区 */
#define MOTOR_ACCEL_RPM_S           50.0f      /* 加/减速度 (RPM/s) */
#define CONTROL_PERIOD_MS           1          /* 梯形计算周期 (ms) */

/* ================================================================
 *  平 移 规 划 器 参 数
 * ================================================================
 */
#define TRANS_MAX_SPEED_MM_S        500.0f     /* 最大线速度 (mm/s) */
#define TRANS_ACCEL_MM_S2           300.0f     /* 线加速度 (mm/s²) */
#define TRANS_MIN_SPEED_MM_S        15.0f      /* 起步最低线速度 (mm/s) */

/* ================================================================
 *  旋 转 规 划 器 参 数 ★新
 * ================================================================
 */
/* 电机能力约束: α_max = MOTOR_ACCEL_RPM_S/60 * (π*D) / (B/2) ≈ 2.68 rad/s² */
#define ROT_MAX_SPEED_RAD_S         1.5708f    /* 最大角速度 ~90°/s (防失步) */
#define ROT_ACCEL_RAD_S2            2.50f      /* 角加速度, 约束在电机能力内 (曾 6.28=2.34×超标) */
#define ROT_MIN_SPEED_RAD_S         0.0873f    /* 起步最低角速度 ~5°/s */

/* ================================================================
 *  航 向 角 度 PID 参 数 (弱修正, 限幅 ±5°/s)
 * ================================================================
 */
#define HEADING_KP                  1.0f
#define HEADING_KI                  0.1f
#define HEADING_DELTA_LIMIT_RAD_S   0.087266f  /* ±5°/s = ±0.087266 rad/s */

/* 由以上参数自动换算 */
#define WHEEL_CIRCUMFERENCE_MM      ((float)WHEEL_DIAMETER_MM * 3.14159265359f)
#define MAX_SPEED_STEPS_S           (MOTOR_MAX_SPEED_RPM * (float)STEPS_PER_REV / 60.0f)
#define MAX_ACCEL_STEPS_S2          (MOTOR_ACCEL_RPM_S * (float)STEPS_PER_REV / 60.0f)
#define MIN_SPEED_STEPS_S           (MOTOR_MIN_SPEED_RPM * (float)STEPS_PER_REV / 60.0f)

/* ================================================================
 *  底 盘 运 动 学 接 口
 * ================================================================
 */

/**
 * @brief  底盘初始化 (内部调用 Motor_Init 绑定左右电机硬件)
 */
void Chassis_Init(void);

/**
 * @brief  绝对位置平移接口 — 使用梯形加减速规划器
 * @param  mm  目标绝对位置 (mm), 正值前进
 *
 * @note   内部将 mm 换算为 steps, 调用 trans_planner 启动规划;
 *         若航向锁定开启, 同步锁定当前 MPU6050 角度为航向目标。
 *
 *         统一管线: trans_planner → v (mm/s)
 *                   rot_planner  → omega_base (rad/s)
 *                   heading_PID  → delta_omega (rad/s)
 *                   差速逆解: v + (omega_base + delta_omega) → 两轮速
 */
void Chassis_MoveTo(float mm);

/**
 * @brief  相对距离平移接口 — 从当前位置前进/后退指定距离
 * @param  mm  相对移动距离 (mm), 正值前进, 负值后退
 *
 * @note   与 MoveTo(绝对) 不同, MoveBy 在当前位置基础上叠加偏移.
 *         适用于状态机/路径规划中"走X毫米"的场景.
 */

/**
 * @brief  绝对角度旋转接口 ★新 — 使用 rot_planner 梯形曲线驱动
 * @param  degrees  目标旋转角度 (度), + 为左转(CCW), - 为右转(CW)
 *
 * @note   内部将 degrees 换算为 rad 并叠加到 rot_planner 的 current_pos,
 *         调用 TrapPlan_SetTarget 启动梯形规划器。
 *
 *         例: Chassis_RotateTo(90.0f) → 原地左转 90° (梯形曲线)
 *              Chassis_RotateTo(-90.0f) → 原地右转 90°
 */
void Chassis_RotateTo(float degrees);

/**
 * @brief  停止所有运动
 * @note   停止 trans_planner 和 rot_planner, 电机减速归零
 */
void Chassis_Stop(void);

/**
 * @brief  1ms 周期调用 — 统一管线更新
 * @note   由 FreeRTOS 软件定时器回调调用
 *
 *         统一管线 (所有模式共用):
 *           1. trans_planner → v
 *           2. rot_planner   → omega_base
 *           3. heading_PID   → delta_omega (限幅 ±5°/s, 不影响规划器)
 *           4. 差速逆解: v + (omega_base + delta_omega) → 两轮速
 */
void Chassis_Update(void);

/**
 * @brief  启用/禁用航向锁定
 * @param  enable  1=启用, 0=禁用
 * @note   启用时, 下次 Chassis_MoveTo 调用时锁当前角度;
 *         航向 PID 以 ±5°/s 弱修正维持锁定。
 */
void Chassis_SetHeadingLock(uint8_t enable);

/* ---- 传感器接口 (陀螺仪数据注入) ---- */

/**
 * @brief  外部传感器数据注入接口
 * @param  gyro_angular_rad_s  当前陀螺仪角速度 (rad/s)
 * @param  angle_z_rad         当前 Z 轴累积角度 (rad)
 */
void Chassis_SetGyroData(float gyro_angular_rad_s, float angle_z_rad);

/* ---- 状态查询接口 (用于任务级状态机) ---- */

/**
 * @brief  查询平移规划器是否完成
 * @retval 1=已完成(TRAP_DONE), 0=仍在运动
 */
uint8_t Chassis_IsMoveDone(void);

/**
 * @brief  查询旋转规划器是否完成
 * @retval 1=已完成(TRAP_DONE), 0=仍在运动
 */
uint8_t Chassis_IsRotateDone(void);

#endif /* __CHASSIS_H__ */