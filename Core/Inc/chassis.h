#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "motor.h"

/* ================================================================
 *  机 械 参 数 — 请根据实际小车填入
 * ================================================================
 */
#define WHEEL_DIAMETER_MM           65.0f      /* 轮子直径 (mm) */
#define WHEEL_BASE_MM               150.0f     /* 两轮间距/轮距 (mm) */
#define STEPS_PER_REV               3200       /* 每转脉冲数 (200步 × 16细分) */

/* ================================================================
 *  运 动 参 数
 * ================================================================
 */
#define MOTOR_MAX_SPEED_RPM         200.0f     /* 最高转速 (RPM) */
#define MOTOR_ACCEL_RPM_S           400.0f     /* 加/减速度 (RPM/s) */
#define CONTROL_PERIOD_MS           1          /* 梯形计算周期 (ms) */

/* 由以上参数自动换算 */
#define WHEEL_CIRCUMFERENCE_MM      ((float)WHEEL_DIAMETER_MM * 3.14159265359f)

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