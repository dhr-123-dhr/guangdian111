#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/* ================================================================
 *  电 机 实 例 结 构 体
 * ================================================================
 */
typedef struct {
    /* ---- 硬件绑定 ---- */
    TIM_HandleTypeDef  *htim;            /* 定时器句柄 (每电机独立通道)        */
    uint32_t            channel;         /* HAL 通道: TIM_CHANNEL_1 ~ 4       */
    GPIO_TypeDef       *dir_port;        /* 方向口端口                        */
    uint16_t            dir_pin;         /* 方向口引脚                        */
    uint8_t             is_advanced_tim; /* 1=高级定时器(需开MOE), 0=通用     */

    /* ---- 速度参数 (统一单位: steps/s) ---- */
    float   current_speed;   /* 当前实际速度 (steps/s), 正=前进, 负=后退  */
    float   target_speed;    /* 目标速度 (steps/s)                       */
    float   max_speed;       /* 最高速度限制 (steps/s)                   */
    float   max_accel;       /* 最大加速度 (steps/s²)                    */

    /* ---- 内部: 速度变化率限幅 ---- */
    float   last_speed;       /* 上一次设定的实际速度值                    */
} Motor_t;

/* ================================================================
 *  全 局 电 机 实 例
 * ================================================================
 */
extern Motor_t g_motor_left;
extern Motor_t g_motor_right;

/* ================================================================
 *  公 共 接 口
 * ================================================================
 */

/**
 * @brief  初始化电机实例 (配置 TIM 为硬件 PWM 模式)
 * @param  motor             电机实例指针
 * @param  htim              定时器句柄 (如 &htim2)
 * @param  channel           HAL 通道宏 (如 TIM_CHANNEL_1)
 * @param  dir_port          方向口 (如 GPIOA)
 * @param  dir_pin           方向引脚 (如 GPIO_PIN_7)
 * @param  max_speed_steps_s 最高速度 (steps/s) — 自动换算自 RPM × 步数/转 ÷ 60
 * @param  max_accel_steps_s2 最大加速度 (steps/s²)
 * @param  is_advanced_tim   1=高级定时器(需开MOE), 0=通用定时器
 */
void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin,
                float max_speed_steps_s, float max_accel_steps_s2,
                uint8_t is_advanced_tim);

/**
 * @brief  设置步进速度 (内部含速率限幅 + RTOS 临界区保护)
 * @param  motor          电机实例
 * @param  steps_per_sec  目标速度 (steps/s), 正值前进, 负值后退, 0=停止
 */
void Stepper_SetSpeed(Motor_t *motor, float steps_per_sec);

/**
 * @brief  设置轮式线速度 (自动换算为 steps/s)
 * @param  motor              电机实例
 * @param  mm_s               轮子线速度 (mm/s)
 * @param  wheel_diameter_mm  轮子直径 (mm)
 * @param  steps_per_rev      每转步数 (200步 × 细分)
 */
void Stepper_SetWheelSpeed(Motor_t *motor, float mm_s,
                           float wheel_diameter_mm, uint32_t steps_per_rev);

/**
 * @brief  获取当前实际速度 (steps/s)
 */
float Motor_GetSpeed(Motor_t *motor);

/**
 * @brief  重置速度变化率限幅状态
 * @param  motor  电机实例
 * @note   清零 last_speed / current_speed / target_speed 并停止 PWM 输出。
 *         用于模式切换时消除速度残留，应在获取互斥锁后调用。
 */
void Stepper_ResetSlew(Motor_t *motor);

#endif /* __MOTOR_H__ */