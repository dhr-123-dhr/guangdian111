#include "motor.h"
#include "tim.h"
#include <math.h>
#include <string.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
/* ================================================================
 *  常 量
 * ================================================================
 */
/* Timer clock after Prescaler: 1 MHz (1,000,000 Hz)
 * STM32F407: APB1 timer clock = 84 MHz, PSC = 83 -> 1 MHz
 *            APB2 timer clock = 84 MHz, PSC = 83 -> 1 MHz
 */
#define TIMER_CLK_HZ         1000000U

/* 最大 ARR 值 (16 位定时器用 65535, 32 位定时器用 0xFFFFFFFF) */
#define ARR_16BIT_MAX        65535U
#define ARR_32BIT_MAX        0xFFFFFFFFU

/* 最小 ARR 值 — 防止频率过高导致占空比无法保证 */
#define ARR_MIN              4U

/* 速度变化率限幅周期 (秒) — 与调用 Stepper_SetSpeed 的频率匹配 */
#define SPEED_SLEW_PERIOD_S  0.001f   /* 1 ms */

/* ================================================================
 *  全 局 电 机 实 例
 * ================================================================
 */
Motor_t g_motor_left;
Motor_t g_motor_right;

/* ================================================================
 *  内 部 辅 助
 * ================================================================
 */

/**
 * @brief  将速度 (steps/s) 换算为 ARR 值
 * @param  steps_per_sec  速度 (steps/s), 绝对值
 * @return ARR 值
 * @note   PWM 频率 = TIMER_CLK_HZ / (ARR + 1)
 *         对于步进脉冲: 1 Hz = 1 step/s
 *         因此 ARR = TIMER_CLK_HZ / steps_per_sec - 1
 */
static uint32_t speed_to_arr(float steps_per_sec)
{
    if (steps_per_sec < 0.01f)
        return ARR_16BIT_MAX;  /* 极低速度 → 最大 ARR → 极低频率 */

    uint32_t arr = (uint32_t)((float)TIMER_CLK_HZ / steps_per_sec) - 1U;

    if (arr < ARR_MIN)
        arr = ARR_MIN;
    if (arr > ARR_16BIT_MAX)
        arr = ARR_16BIT_MAX;

    return arr;
}

/**
 * @brief  限幅
 */
static float clampf(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
 * @brief  写硬件寄存器: 设置 ARR, CCR 和 DIR
 * @param  motor             电机实例
 * @param  steps_per_sec     速度绝对值 (>= 0)
 * @param  direction         方向: +1 正转, -1 反转
 * @param  stop              1 = 停止 (CCR=0 关脉冲)
 *
 * @note   调用者需保证已在临界区内
 */
static void motor_write_hw(Motor_t *motor, float steps_per_sec,
                           int8_t direction, uint8_t stop)
{
    if (stop) {
        /* ---- 停止: CCR = 0, 利用输出比较预装载保证最后脉冲完整 ---- */
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
    } else {
        /* ---- DIR 引脚 ---- */
        if (direction >= 0)
            HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);

        uint32_t arr = speed_to_arr(steps_per_sec);
        uint32_t ccr = arr / 2U;   /* 50% 占空比 */

        __HAL_TIM_SET_AUTORELOAD(motor->htim, arr);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, ccr);
    }
}

/* ================================================================
 *  公 共 接 口
 * ================================================================
 */

/**
 * @brief  初始化电机实例
 */
void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin,
                float max_speed_steps_s, float max_accel_steps_s2,
                uint8_t is_advanced_tim)
{
    memset(motor, 0, sizeof(Motor_t));

    motor->htim           = htim;
    motor->channel        = channel;
    motor->dir_port       = dir_port;
    motor->dir_pin        = dir_pin;
    motor->is_advanced_tim = is_advanced_tim;
    motor->max_speed      = max_speed_steps_s;
    motor->max_accel      = max_accel_steps_s2;

    /* ---- TIM 基础参数和 PWM 通道已由 CubeMX 在 main() 中完成 ----
     *  (MX_TIMx_Init → HAL_TIM_PWM_Init + ConfigChannel + MspPostInit)
     *  这里仅负责启动 PWM 和配置 DIR 引脚，避免重复初始化导致冲突。
     */

    /* ---- 高级定时器需开启 MOE (主输出使能) ---- */
    if (is_advanced_tim) {
        htim->Instance->BDTR |= TIM_BDTR_MOE;
    }

    /* ---- DIR 引脚配置 ---- */
    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin   = dir_pin;
    gpio_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull  = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(dir_port, &gpio_cfg);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);

    /* ---- 启动 PWM 通道 (脉冲输出由 CCR=0 初始控制为低) ---- */
    HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief  设置步进速度 (核心接口)
 *
 * 策略:
 *   1. 目标限幅
 *   2. 速度变化率限幅 (基于 last_speed, 限制每次最大变化量)
 *   3. RTOS 临界区保护 ARR/CCR/DIR 写操作
 *   4. 停止时 CCR = 0 (利用 OC 预装载, 最后脉冲完整)
 */
void Stepper_SetSpeed(Motor_t *motor, float steps_per_sec)
{
    /* ---- 1. 限幅 ---- */
    steps_per_sec = clampf(steps_per_sec, -motor->max_speed, motor->max_speed);
    motor->target_speed = steps_per_sec;

    /* ---- 2. 速度变化率限幅 ---- */
    float delta = steps_per_sec - motor->last_speed;
    float max_delta = motor->max_accel * SPEED_SLEW_PERIOD_S;

    delta = clampf(delta, -max_delta, max_delta);
    float new_speed = motor->last_speed + delta;

    /* ---- 3. 临界区保护 ---- */
    taskENTER_CRITICAL();
    {
        motor->current_speed = new_speed;
        motor->last_speed    = new_speed;

        if (fabsf(new_speed) < 0.01f) {
            /* 停止: CCR=0, 脉冲在下一周期自动消失 */
            motor_write_hw(motor, 0.0f, 1, 1);
        } else {
            int8_t dir = (new_speed >= 0.0f) ? 1 : -1;
            motor_write_hw(motor, fabsf(new_speed), dir, 0);
        }
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  设置轮式线速度 (mm/s → steps/s)
 */
void Stepper_SetWheelSpeed(Motor_t *motor, float mm_s,
                           float wheel_diameter_mm, uint32_t steps_per_rev)
{
    float circumference = 3.14159265359f * wheel_diameter_mm;
    float steps_per_sec = mm_s / circumference * (float)steps_per_rev;
    Stepper_SetSpeed(motor, steps_per_sec);
}

/**
 * @brief  获取当前实际速度 (steps/s)
 */
float Motor_GetSpeed(Motor_t *motor)
{
    return motor->current_speed;
}

/**
 * @brief  重置速度变化率限幅状态
 * @note   清零 last_speed / current_speed / target_speed 并停止 PWM 输出。
 *         用于模式切换时消除速度残留。
 */
void Stepper_ResetSlew(Motor_t *motor)
{
    taskENTER_CRITICAL();
    {
        motor->last_speed    = 0.0f;
        motor->current_speed = 0.0f;
        motor->target_speed  = 0.0f;
        motor_write_hw(motor, 0.0f, 1, 1);  /* 停止 PWM 输出 */
    }
    taskEXIT_CRITICAL();
}