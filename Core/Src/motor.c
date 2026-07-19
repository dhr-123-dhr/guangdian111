#include "motor.h"
#include <math.h>
#include <string.h>

/* ========== 系统时钟相关宏 ========== */
#define TIM_CLK_HZ           1000000U   /* TIM2/TIM3在APB1总线上，84MHz -> 不分频时timer clk = 168MHz/2 = 84MHz
                                           但这里用Prescaler=0使timer clk=84MHz来算脉冲，单位Hz */
/* TIM2/TIM3时钟源：挂载APB1=42MHz, APB1 timer clk为84MHz (APB1*2) */
/* 为简化，电机初始化时设置Prescaler=0，所以TIM clk = 84MHz */
#define TIMER_FREQ           84000000U  /* TIM2/TIM3 clock = 84MHz (HCLK/2 * 2 = 168MHz/2 *2 ) */

#define TICK_MS_TO_SEC(ms)   ((float)(ms) / 1000.0f)

/* ========== 全局电机实例 ========== */
Motor_t g_motor_left;
Motor_t g_motor_right;

/* ========== 内部辅助 ========== */

/**
 * @brief  将 RPM 转速转换为定时器半周期 tick 数
 * @param  rpm  转速 (RPM), 绝对值
 * @return 半周期所需的 timer clock 计数值
 * @note   每转 3200 微步 (200步 × 16细分)
 *         脉冲频率 f = rpm / 60 * 3200 (Hz)
 *         半周期时间 = 1/(2*f) = 1/(2 * rpm/60 * 3200) = 30/(rpm*3200) 秒
 *         半周期 tick = TIMER_FREQ * 半周期时间 = TIMER_FREQ * 30 / (rpm * 3200)
 */
static uint32_t rpm_to_half_period(float rpm)
{
    if (rpm < 0.01f) rpm = 0.01f;      /* 防除零 */
    float half_period_us = 30.0f * 1000000.0f / (rpm * 3200.0f);
    uint32_t ticks = (uint32_t)((float)TIMER_FREQ * half_period_us / 1000000.0f);
    if (ticks < 2) ticks = 2;           /* 最低限制，防止频率过高 */
    return ticks;
}

/**
 * @brief  根据速度方向设置DIR引脚
 */
static void motor_set_dir(Motor_t *motor, int8_t direction)
{
    if (direction >= 0)
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
}

/**
 * @brief  启动定时器输出比较中断（从停止状态恢复）
 */
static void motor_start_timer(Motor_t *motor)
{
    if (!motor->is_running) {
        motor->is_running = 1;
        motor->toggle_count = 0;

        /* 设初始比较值 = CNT + half_period (确保第一次中断在半个周期后) */
        uint32_t cnt = __HAL_TIM_GET_COUNTER(motor->htim);
        *motor->ccr_reg = cnt + motor->half_period_ticks;

        /* 清除标志并启动定时器OC中断 */
        __HAL_TIM_CLEAR_IT(motor->htim, motor->channel_hal == TIM_CHANNEL_1 ? TIM_IT_CC1 :
                                          motor->channel_hal == TIM_CHANNEL_2 ? TIM_IT_CC2 :
                                          motor->channel_hal == TIM_CHANNEL_3 ? TIM_IT_CC3 : TIM_IT_CC4);
        __HAL_TIM_ENABLE_IT(motor->htim, motor->channel_hal == TIM_CHANNEL_1 ? TIM_IT_CC1 :
                                         motor->channel_hal == TIM_CHANNEL_2 ? TIM_IT_CC2 :
                                         motor->channel_hal == TIM_CHANNEL_3 ? TIM_IT_CC3 : TIM_IT_CC4);
        HAL_TIM_OC_Start_IT(motor->htim, motor->channel_hal);
    }
}

/**
 * @brief  停止定时器输出比较中断（电机停止）
 */
static void motor_stop_timer(Motor_t *motor)
{
    if (motor->is_running) {
        HAL_TIM_OC_Stop_IT(motor->htim, motor->channel_hal);
        __HAL_TIM_DISABLE_IT(motor->htim, motor->channel_hal == TIM_CHANNEL_1 ? TIM_IT_CC1 :
                                           motor->channel_hal == TIM_CHANNEL_2 ? TIM_IT_CC2 :
                                           motor->channel_hal == TIM_CHANNEL_3 ? TIM_IT_CC3 : TIM_IT_CC4);
        motor->is_running = 0;

        /* 输出强制为低 */
        if (motor->channel_hal == TIM_CHANNEL_1)
            motor->htim->Instance->CCER &= ~TIM_CCER_CC1E;
        else if (motor->channel_hal == TIM_CHANNEL_2)
            motor->htim->Instance->CCER &= ~TIM_CCER_CC2E;
        /* ... 按需扩展其他通道 */
    }
}

/* ========== 公共接口 ========== */

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel_hal,
                GPIO_TypeDef *dir_port, uint16_t dir_pin,
                float max_speed, float accel)
{
    memset(motor, 0, sizeof(Motor_t));

    motor->htim        = htim;
    motor->channel_hal = channel_hal;
    motor->dir_port    = dir_port;
    motor->dir_pin     = dir_pin;
    motor->max_speed_rpm = max_speed;
    motor->accel_rpm_per_s = accel;
    motor->state       = MOTOR_STOP;

    /* 根据通道获取CCRx寄存器指针 */
    if (channel_hal == TIM_CHANNEL_1)
        motor->ccr_reg = &htim->Instance->CCR1;
    else if (channel_hal == TIM_CHANNEL_2)
        motor->ccr_reg = &htim->Instance->CCR2;
    else if (channel_hal == TIM_CHANNEL_3)
        motor->ccr_reg = &htim->Instance->CCR3;
    else if (channel_hal == TIM_CHANNEL_4)
        motor->ccr_reg = &htim->Instance->CCR4;

    /* ---- 重新配置 TIM 为输出比较翻转模式 (OC Toggle) ---- */
    /* 先停掉可能已在运行的PWM */
    HAL_TIM_PWM_Stop(htim, channel_hal);

    /* 设置 Prescaler = 0, Period = 0xFFFF (32bit timer得用0xFFFFFFFF) */
    htim->Init.Prescaler = 0;
    /* TIM2/TIM3是16位定时器，Period最大65535 */
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_OC_Init(htim);

    /* 配置输出比较通道为翻转模式 */
    TIM_OC_InitTypeDef oc_config = {0};
    oc_config.OCMode   = TIM_OCMODE_TOGGLE;
    oc_config.Pulse    = 0;                    /* 初始比较值，运行中动态更新 */
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(htim, &oc_config, channel_hal);

    /* 方向引脚初始化为输出低 */
    GPIO_InitTypeDef gpio_cfg = {0};
    gpio_cfg.Pin   = dir_pin;
    gpio_cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull  = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(dir_port, &gpio_cfg);
    HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);

    motor->half_period_ticks = rpm_to_half_period(1.0f); /* 低速占位 */
}

void Motor_SetSpeed(Motor_t *motor, float target_rpm)
{
    /* 限幅 */
    if (target_rpm > motor->max_speed_rpm)
        target_rpm = motor->max_speed_rpm;
    else if (target_rpm < -motor->max_speed_rpm)
        target_rpm = -motor->max_speed_rpm;

    motor->target_speed_rpm = target_rpm;
}

float Motor_GetSpeed(Motor_t *motor)
{
    return motor->current_speed_rpm;
}

int32_t Motor_GetSteps(Motor_t *motor)
{
    return motor->position;
}

void Motor_Tick(Motor_t *motor)
{
    float current  = motor->current_speed_rpm;
    float target   = motor->target_speed_rpm;
    float step     = motor->accel_rpm_per_s * TICK_MS_TO_SEC(1); /* 每ms加速度增量 */
    float new_speed = current;
    MotorState_t new_state = motor->state;

    /* ---- 方向判断 ---- */
    int8_t cur_dir  = (current >= 0) ? 1 : -1;
    int8_t tgt_dir  = (target >= 0) ? 1 : -1;

    if (fabsf(target) < 0.01f) {
        /* 目标为零 → 开始减速 → 停止 */
        if (fabsf(current) > 0.01f) {
            float decel = fabsf(current) - step;
            if (decel <= 0.01f) {
                new_speed = 0.0f;
                new_state = MOTOR_STOP;
            } else {
                new_speed = (cur_dir >= 0) ? decel : -decel;
                new_state = MOTOR_DECEL;
            }
        } else {
            new_speed = 0.0f;
            new_state = MOTOR_STOP;
        }
    } else {
        /* 目标非零 */
        if (fabsf(current) < 0.01f) {
            /* 从零启动 → 加速 */
            new_speed = (tgt_dir >= 0) ? step : -step;
            new_state = MOTOR_ACCEL;
        } else if (cur_dir == tgt_dir) {
            /* 同向 */
            float diff = fabsf(target) - fabsf(current);
            if (diff > step) {
                /* 加速 */
                new_speed = current + step * tgt_dir;
                new_state = MOTOR_ACCEL;
            } else if (diff < -step) {
                /* 减速 */
                new_speed = current - step * cur_dir;
                new_state = MOTOR_DECEL;
            } else {
                /* 接近目标 → 匀速 */
                new_speed = target;
                new_state = MOTOR_CONST;
            }
        } else {
            /* 反向 → 先减速过零 */
            float decel = fabsf(current) - step;
            if (decel <= 0.01f) {
                /* 过零 */
                new_speed = (tgt_dir >= 0) ? step : -step;
                new_state = MOTOR_ACCEL;
            } else {
                new_speed = (cur_dir >= 0) ? decel : -decel;
                new_state = MOTOR_DECEL;
            }
        }
    }

    motor->current_speed_rpm = new_speed;
    motor->state = new_state;

    /* ---- 根据速度更新脉冲频率 ---- */
    if (new_state == MOTOR_STOP) {
        motor_stop_timer(motor);
    } else {
        uint32_t hp = rpm_to_half_period(fabsf(new_speed));
        motor->half_period_ticks = hp;
        motor_set_dir(motor, (new_speed >= 0) ? 1 : -1);
        /* 更改方向引脚后需短暂延时让DIR信号稳定 */
        motor_start_timer(motor);
    }
}

/**
 * @brief  TIM 输出比较中断回调 — 产生步进脉冲翻转
 *
 * 策略: 每次OC匹配触发中断后:
 *       1. CCR += half_period → 预定下一次翻转时刻
 *       2. 硬件自动翻转引脚电平 (OC TOGGLE模式)
 *       3. 每2次翻转 = 1个完整脉冲, position +1
 */
void Motor_StepIRQHandler(Motor_t *motor)
{
    if (!motor->is_running) return;

    /* 更新下一次比较值 = 当前CCR + 半周期 */
    uint32_t ccr = *motor->ccr_reg;
    *motor->ccr_reg = ccr + motor->half_period_ticks;

    /* 统计步数: 每两次翻转(上升+下降) = 一个完整脉冲 = 1微步 */
    motor->toggle_count++;
    if (motor->toggle_count >= 2) {
        motor->toggle_count = 0;
        if (motor->current_speed_rpm >= 0)
            motor->position++;
        else
            motor->position--;
    }
}