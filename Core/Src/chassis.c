#include "chassis.h"
#include <math.h>
#include "tim.h"

/* ---- 内部变量 ---- */
static float g_gyro_angular_speed = 0.0f;  /* 陀螺仪角速度 (rad/s), 闭环预留 */
static float g_target_left_mm_s  = 0.0f;   /* 左轮目标线速度 (mm/s) */
static float g_target_right_mm_s = 0.0f;   /* 右轮目标线速度 (mm/s) */

/* ---- 梯形规划器实例 ---- */
static TrapezoidPlanner_t g_planner_left;
static TrapezoidPlanner_t g_planner_right;

/* ---- 模式标志: 0=速度模式, 1=位置模式 ---- */
static uint8_t g_position_mode = 0;

/**
 * @brief  限幅
 */
static float clampf(float val, float min, float max)
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
    /* ---- 将用户友好的 RPM 参数换算为 steps/s 和 steps/s² ---- */
    float max_speed = MOTOR_MAX_SPEED_RPM * (float)STEPS_PER_REV / 60.0f;
    float max_accel = MOTOR_ACCEL_RPM_S * (float)STEPS_PER_REV / 60.0f;
    float min_speed = MIN_SPEED_STEPS_S;  /* 由 MOTOR_MIN_SPEED_RPM 自动换算 */

    /* ---- 电机A: 左轮 ---- */
    /* STP: PA5 = TIM2_CH1, DIR: PA7  (TIM2 是通用定时器, is_advanced_tim=0) */
    Motor_Init(&g_motor_left, &htim2, TIM_CHANNEL_1,
               GPIOA, GPIO_PIN_7,
               max_speed, max_accel, 0);

    /* ---- 电机B: 右轮 ---- */
    /* STP: PA6 = TIM3_CH1, DIR: PC4  (TIM3 是通用定时器, is_advanced_tim=0) */
    Motor_Init(&g_motor_right, &htim3, TIM_CHANNEL_1,
               GPIOC, GPIO_PIN_4,
               max_speed, max_accel, 0);

    /* ---- 梯形规划器初始化 ---- */
    TrapPlan_Init(&g_planner_left,  max_speed, min_speed, max_accel);
    TrapPlan_Init(&g_planner_right, max_speed, min_speed, max_accel);
}

void Chassis_SetMotion(float linear_mm_s, float angular_rad_s)
{
    /* ---- 切换到速度模式, 清除位置模式 ---- */
    g_position_mode = 0;
    TrapPlan_Stop(&g_planner_left);
    TrapPlan_Stop(&g_planner_right);

    /* ---- 差速运动学解算 ---- */
    /* V_left  = V_linear - ω × (wheel_base / 2)     */
    /* V_right = V_linear + ω × (wheel_base / 2)     */
    float half_base = WHEEL_BASE_MM / 2.0f;

    float V_left_mm_s  = linear_mm_s - angular_rad_s * half_base;
    float V_right_mm_s = linear_mm_s + angular_rad_s * half_base;

    /* 限幅到车轮最大线速度 */
    float max_wheel_mm_s  = MOTOR_MAX_SPEED_RPM * WHEEL_CIRCUMFERENCE_MM / 60.0f;
    V_left_mm_s  = clampf(V_left_mm_s,  -max_wheel_mm_s, max_wheel_mm_s);
    V_right_mm_s = clampf(V_right_mm_s, -max_wheel_mm_s, max_wheel_mm_s);

    /* 仅存储目标 — 实际速度由 Chassis_Update 按 1ms 周期渐进逼近 */
    g_target_left_mm_s  = V_left_mm_s;
    g_target_right_mm_s = V_right_mm_s;

    /* TODO: 后续接入陀螺仪闭环时，在此处用 g_gyro_angular_speed 做反馈修正 */
    (void)g_gyro_angular_speed;
}

void Chassis_Stop(void)
{
    /* ---- 切换到速度模式并置零目标 ---- */
    g_position_mode = 0;
    TrapPlan_Stop(&g_planner_left);
    TrapPlan_Stop(&g_planner_right);
    g_target_left_mm_s  = 0.0f;
    g_target_right_mm_s = 0.0f;

    /* 立即触发一次 ramp-down: Stepper_SetSpeed(0) 内部 slew limiter
     * 会从当前速度按 max_accel 逐步减速到零, 不再直接切断 PWM */
    Stepper_SetSpeed(&g_motor_left,  0.0f);
    Stepper_SetSpeed(&g_motor_right, 0.0f);
}

/**
 * @brief  绝对位置移动接口 — 使用梯形加减速规划器
 *
 * mm → steps 换算: steps = mm / circumference * steps_per_rev
 *
 * 规划器由 Chassis_Update 每 1ms 驱动:
 *   加速段 → 匀速段 → 减速段 → DONE
 *   短距离自动降级为三角形曲线
 */
void Chassis_MoveTo(float left_mm, float right_mm)
{
    /* ---- 切换到位置模式, 清除速度模式 ---- */
    g_position_mode = 1;
    g_target_left_mm_s  = 0.0f;
    g_target_right_mm_s = 0.0f;

    /* mm → steps */
    float steps_per_mm = (float)STEPS_PER_REV / WHEEL_CIRCUMFERENCE_MM;
    float left_steps   = left_mm  * steps_per_mm;
    float right_steps  = right_mm * steps_per_mm;

    TrapPlan_SetTarget(&g_planner_left,  left_steps);
    TrapPlan_SetTarget(&g_planner_right, right_steps);
}

/**
 * @brief  1ms 周期调用 — 双模式驱动
 * @note   由 FreeRTOS 软件定时器回调调用
 *
 * 位置模式 (g_position_mode=1):
 *   梯形规划器 → TrapPlan_Update → Stepper_SetSpeed
 *   完全走梯形曲线, 精确到达目标位置后停止
 *
 * 速度模式 (g_position_mode=0):
 *   Stepper_SetWheelSpeed → Stepper_SetSpeed
 *   Stepper_SetSpeed 内部有 per-call slew rate limit (1ms 一次),
 *   向目标速度渐进逼近, 无需额外梯形状态机
 */
void Chassis_Update(void)
{
    if (g_position_mode) {
        /* ============================================================
         *  位置模式: 梯形规划器驱动
         * ============================================================ */
        uint8_t left_done  = TrapPlan_IsDone(&g_planner_left);
        uint8_t right_done = TrapPlan_IsDone(&g_planner_right);

        if (!left_done) {
            float speed = TrapPlan_Update(&g_planner_left, 0.001f);
            Stepper_SetSpeed(&g_motor_left, speed);
        } else {
            /* 已到达, 确保速度归零 */
            Stepper_SetSpeed(&g_motor_left, 0.0f);
        }

        if (!right_done) {
            float speed = TrapPlan_Update(&g_planner_right, 0.001f);
            Stepper_SetSpeed(&g_motor_right, speed);
        } else {
            /* 已到达, 确保速度归零 */
            Stepper_SetSpeed(&g_motor_right, 0.0f);
        }

        /* 两个都完成后自动切换回速度模式 */
        if (left_done && right_done) {
            g_position_mode = 0;
        }
    } else {
        /* ============================================================
         *  速度模式: 渐进逼近
         * ============================================================ */
        /* 每次 1ms 都向目标速度逼近一个小增量 */
        Stepper_SetWheelSpeed(&g_motor_left,  g_target_left_mm_s,
                              WHEEL_DIAMETER_MM, STEPS_PER_REV);
        Stepper_SetWheelSpeed(&g_motor_right, g_target_right_mm_s,
                              WHEEL_DIAMETER_MM, STEPS_PER_REV);
    }
}

void Chassis_SetGyroAngularSpeed(float gyro_angular_rad_s)
{
    g_gyro_angular_speed = gyro_angular_rad_s;
}