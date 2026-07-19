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
#define MOTOR_ACCEL_RPM_S          50.0f     /* 加/减速度 (RPM/s) */
#define CONTROL_PERIOD_MS           1          /* 梯形计算周期 (ms) */

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
 * @brief  通用运动控制接口
 * @param  linear_mm_s   线速度 (mm/s), 正值前进
 * @param  angular_rad_s 角速度 (rad/s), 正值左转 (CCW)
 *
 * @note   差速解算:
 *         V_left  = (linear - angular * wheel_base/2)
 *         V_right = (linear + angular * wheel_base/2)
 *         再由 V_mm_s 换算为电机 RPM
 */
void Chassis_SetMotion(float linear_mm_s, float angular_rad_s);

/**
 * @brief  停止 (等效 Chassis_SetMotion(0, 0))
 */
void Chassis_Stop(void);

/**
 * @brief  绝对位置移动接口 — 使用梯形加减速规划器
 * @param  left_mm   左轮目标绝对位置 (mm)
 * @param  right_mm  右轮目标绝对位置 (mm)
 *
 * @note   内部将 mm 换算为 steps, 调用 TrapPlan_SetTarget 启动规划;
 *         规划器由 Chassis_Update 每 1ms 驱动, 自动完成加减速。
 */
void Chassis_MoveTo(float left_mm, float right_mm);

/**
 * @brief  原地旋转指定角度 (位置模式)
 * @param  degrees  旋转角度 (度), + 为左转(CCW), - 为右转(CW)
 *
 * @note   差速解算:
 *         弧长 = angle_rad × (wheel_base / 2)
 *         左轮后退 arc_mm, 右轮前进 arc_mm
 *         内部自动获取当前规划器位置并叠加目标, 调用 Chassis_MoveTo 执行
 *
 *         例: Chassis_Rotate(90.0f) → 原地左转 90°
 *              Chassis_Rotate(-90.0f) → 原地右转 90°
 */
void Chassis_Rotate(float degrees);

/**
 * @brief  1ms 周期调用 — 更新梯形加减速状态机
 * @note   由 FreeRTOS 软件定时器回调调用
 */
void Chassis_Update(void);

/* ---- 传感器接口 (预留陀螺仪闭环) ---- */

/**
 * @brief  外部传感器数据注入接口 (后续陀螺仪闭环时使用)
 * @param  gyro_angular_rad_s  当前陀螺仪角速度 (rad/s)
 */
void Chassis_SetGyroAngularSpeed(float gyro_angular_rad_s);

#endif /* __CHASSIS_H__ */