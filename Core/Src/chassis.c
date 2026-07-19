#include "chassis.h"
#include <math.h>
#include "tim.h"

/* ---- 内部变量 ---- */
static float g_gyro_angular_speed = 0.0f;  /* 陀螺仪角速度 (rad/s), 闭环预留 */

/**
 * @brief  线速度 (mm/s) → 电机转速 (RPM)
 * @param  mm_s  轮子线速度 (mm/s)
 * @return 电机 RPM
 * @note   RPM = mm_s / (π × D_mm) × 60
 */
static float linear_to_rpm(float mm_s)
{
    return mm_s / WHEEL_CIRCUMFERENCE_MM * 60.0f;
}

/**
 * @brief  限幅
 */
static float clamp(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/* ================================================================
 *  公 共 接 口
 * ================================================================
 */

void Chassis_Init(void)
{
    /* ---- 电机A: 左轮 ---- */
    /* STP: PA5 = TIM2_CH1, DIR: PA7 */
    Motor_Init(&g_motor_left, &htim2, TIM_CHANNEL_1,
               GPIOA, GPIO_PIN_7,
               MOTOR_MAX_SPEED_RPM, MOTOR_ACCEL_RPM_S);

    /* ---- 电机B: 右轮 ---- */
    /* STP: PA6 = TIM3_CH1, DIR: PC4 */
    Motor_Init(&g_motor_right, &htim3, TIM_CHANNEL_1,
               GPIOC, GPIO_PIN_4,
               MOTOR_MAX_SPEED_RPM, MOTOR_ACCEL_RPM_S);
}

void Chassis_SetMotion(float linear_mm_s, float angular_rad_s)
{
    /* ---- 差速运动学解算 ---- */
    /* V_left  = V_linear - ω × (wheel_base / 2)     */
    /* V_right = V_linear + ω × (wheel_base / 2)     */
    float half_base = WHEEL_BASE_MM / 2.0f;

    float V_left_mm_s  = linear_mm_s - angular_rad_s * half_base;
    float V_right_mm_s = linear_mm_s + angular_rad_s * half_base;

    /* 转电机 RPM */
    float left_rpm  = linear_to_rpm(V_left_mm_s);
    float right_rpm = linear_to_rpm(V_right_mm_s);

    /* 限幅 */
    left_rpm  = clamp(left_rpm,  -MOTOR_MAX_SPEED_RPM, MOTOR_MAX_SPEED_RPM);
    right_rpm = clamp(right_rpm, -MOTOR_MAX_SPEED_RPM, MOTOR_MAX_SPEED_RPM);

    /* 下达目标速度 */
    Motor_SetSpeed(&g_motor_left,  left_rpm);
    Motor_SetSpeed(&g_motor_right, right_rpm);

    /* TODO: 后续接入陀螺仪闭环时，在此处用 g_gyro_angular_speed 做反馈修正 */
    (void)g_gyro_angular_speed;
}

void Chassis_Stop(void)
{
    Chassis_SetMotion(0.0f, 0.0f);
}

void Chassis_Update(void)
{
    /* 1ms 一次，更新两个电机的梯形加减速状态机 */
    Motor_Tick(&g_motor_left);
    Motor_Tick(&g_motor_right);
}

void Chassis_SetGyroAngularSpeed(float gyro_angular_rad_s)
{
    g_gyro_angular_speed = gyro_angular_rad_s;
}