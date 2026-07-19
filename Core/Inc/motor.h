#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/* 电机状态枚举 */
typedef enum {
    MOTOR_STOP = 0,      /* 停止 */
    MOTOR_ACCEL,          /* 加速段 */
    MOTOR_CONST,          /* 匀速段 */
    MOTOR_DECEL           /* 减速段 */
} MotorState_t;

/* 电机实例结构体 */
typedef struct {
    /* ---- 硬件绑定 ---- */
    TIM_HandleTypeDef   *htim;           /* 定时器句柄 (TIM2/TIM3) */
    uint32_t             channel_hal;    /* HAL通道宏: TIM_CHANNEL_1 ~ 4 */
    volatile uint32_t   *ccr_reg;       /* CCRx 寄存器指针 (直接操作) */
    GPIO_TypeDef        *dir_port;      /* 方向口端口 */
    uint16_t             dir_pin;        /* 方向口引脚 */

    /* ---- 速度参数 ---- */
    float   current_speed_rpm;   /* 当前实际转速 (RPM) */
    float   target_speed_rpm;    /* 目标转速 (RPM), >0正转 <0反转 */
    float   accel_rpm_per_s;     /* 加减速度 (RPM/s) */
    float   max_speed_rpm;       /* 最高转速限制 */

    /* ---- 位置 ---- */
    volatile int32_t position;   /* 累计微步数 (带方向) */

    /* ---- 内部状态 ---- */
    MotorState_t state;           /* 梯形加减速状态机 */
    uint8_t      is_running;     /* 定时器是否已启动 */
    volatile uint32_t half_period_ticks; /* 半周期定时器tick数 */
    uint8_t       toggle_count;  /* 翻转计数(区分上升/下降沿) */
} Motor_t;

/* 全局电机实例 */
extern Motor_t g_motor_left;
extern Motor_t g_motor_right;

/* ========== 电机层接口 ========== */

/**
 * @brief  初始化电机实例 (绑定硬件 + 重配TIM为输出比较翻转模式)
 * @param  motor       电机实例指针
 * @param  htim        定时器句柄 (如 &htim2)
 * @param  channel_hal HAL通道宏 (如 TIM_CHANNEL_1)
 * @param  dir_port    方向口 (如 GPIOA)
 * @param  dir_pin     方向引脚 (如 GPIO_PIN_7)
 * @param  max_speed   最高转速限制 (RPM)
 * @param  accel       加/减速度 (RPM/s)
 */
void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel_hal,
                GPIO_TypeDef *dir_port, uint16_t dir_pin,
                float max_speed, float accel);

/**
 * @brief  设置目标转速 (自动进行梯形加减速)
 * @param  motor      电机实例
 * @param  target_rpm 目标转速 (RPM), 正=前进 负=后退
 */
void Motor_SetSpeed(Motor_t *motor, float target_rpm);

/** @brief 获取当前实际转速 (RPM) */
float Motor_GetSpeed(Motor_t *motor);

/** @brief 获取累计微步数 (带符号) */
int32_t Motor_GetSteps(Motor_t *motor);

/**
 * @brief  1ms 周期调用 — 梯形加减速状态机更新
 * @note   由 FreeRTOS 软件定时器或外部1ms定时器ISR调用
 */
void Motor_Tick(Motor_t *motor);

/**
 * @brief  定时器输出比较中断回调 — 产生步进脉冲
 * @note   由 HAL_TIM_OC_DelayElapsedCallback 中根据htim->Instance分发调用
 */
void Motor_StepIRQHandler(Motor_t *motor);

#endif /* __MOTOR_H__ */