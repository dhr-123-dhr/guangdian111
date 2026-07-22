#include "chassis.h"
#include "mpu6050.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* ================================================================
 *  辅 助 宏
 * ================================================================
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD(d)  ((d) * 0.01745329252f)
#define RAD2DEG(r)  ((r) * 57.2957795131f)

#define CLAMP(v, lo, hi)  (((v) < (lo)) ? (lo) : ((v) > (hi)) ? (hi) : (v))

/* mm <-> steps 换算 */
#define MM2STEPS(mm)     ((mm) * (float)STEPS_PER_REV / WHEEL_CIRCUMFERENCE_MM)
#define STEPS2MM(steps)  ((steps) * WHEEL_CIRCUMFERENCE_MM / (float)STEPS_PER_REV)

/* ================================================================
 *  内 部 状 态
 * ================================================================
 */

/* ---- 两个梯形规划器 ---- */
static TrapezoidPlanner_t g_trans_planner;   /* 平移规划器: current_pos 单位 steps */
static TrapezoidPlanner_t g_rot_planner;     /* 旋转规划器: current_pos 单位 rad (以 steps 形式存储, 量纲统一) */

/* ---- 航向PID ---- */
static float g_heading_target_rad = 0.0f;    /* 目标航向角 (rad) */
static float g_heading_integral   = 0.0f;    /* PI 积分项 */
static uint8_t g_heading_locked   = 0;       /* 1=航向锁定生效 */

/* ---- 陀螺仪数据缓存 ---- */
static float g_gyro_angular_rad_s = 0.0f;    /* 当前 Z 轴角速度 (rad/s) */
static float g_gyro_angle_z_rad   = 0.0f;    /* 当前 Z 轴累积角度 (rad) */

/* ---- 保持 Chassis_SetGyroAngularSpeed 的后向兼容 ---- */
/* 已由 Chassis_SetGyroData 替代, 保留旧接口兼容 freertos.c 旧调用 */

/* ================================================================
 *  初 始 化
 * ================================================================
 */
void Chassis_Init(void)
{
    /* ---- 电机层初始化 ---- */
    /* 通过 Stepper_SetWheelSpeed 设置目标速度: mm/s 自动换算为 steps/s */
    Motor_Init(&g_motor_left,  &htim2, TIM_CHANNEL_1,
               GPIOA, GPIO_PIN_7,
               MAX_SPEED_STEPS_S, MAX_ACCEL_STEPS_S2, 0);
    Motor_Init(&g_motor_right, &htim3, TIM_CHANNEL_1,
               GPIOC, GPIO_PIN_4,
               MAX_SPEED_STEPS_S, MAX_ACCEL_STEPS_S2, 0);

    /* ---- 平移规划器 ---- */
    TrapPlan_Init(&g_trans_planner,
                  MAX_SPEED_STEPS_S, MIN_SPEED_STEPS_S, MAX_ACCEL_STEPS_S2);

    /* ---- 旋转规划器 ★新 ---- */
    /* rot_planner 的"位置" 单位是 rad, 但 trapplan 用 float steps 统一处理。
     * 这里 max_speed = 最大角速度 rad/s, min_speed = 最小角速度 rad/s,
     * max_accel = 角加速度 rad/s², 把它们当作虚拟的 "steps/s" 和 "steps/s²" 传入.
     *
     * 效果: TrapPlan_Update 输出的"速度"就是 rad/s, "位置"就是 rad. */
    TrapPlan_Init(&g_rot_planner,
                  ROT_MAX_SPEED_RAD_S, ROT_MIN_SPEED_RAD_S, ROT_ACCEL_RAD_S2);
    g_rot_planner.epsilon = 0.0005f;  /* rad 模式: ~0.029°, 支持小角度旋转 */

    /* ---- 航向PID ---- */
    g_heading_locked      = 0;
    g_heading_target_rad  = 0.0f;
    g_heading_integral    = 0.0f;

    /* ---- 陀螺仪缓存清零 ---- */
    g_gyro_angular_rad_s = 0.0f;
    g_gyro_angle_z_rad   = 0.0f;
}

/* ================================================================
 *  平 移 接 口 — 锁航向
 * ================================================================
 */


/* ---- 相对距离平移 ---- */
void Chassis_Moveto(float mm)
{
    float delta_steps = MM2STEPS(mm);
    float target_steps = g_trans_planner.current_pos + delta_steps;

    /* 停止旋转规划器 */
    TrapPlan_Stop(&g_rot_planner);

    TrapPlan_SetTarget(&g_trans_planner, target_steps);

    /* 航向目标由 SetHeadingLock 快照, Moveto 不再覆写, 确保 PID 持续修正 */
}

/* ================================================================
 *  旋 转 接 口 ★新 — rot_planner 梯形曲线驱动
 * ================================================================
 */
void Chassis_RotateTo(float degrees)
{
    float delta_rad = DEG2RAD(degrees);
    float target_rad = g_rot_planner.current_pos + delta_rad;

    TrapPlan_SetTarget(&g_rot_planner, target_rad);

    /* 航向目标在 Update 中实时从 rot_planner.current_pos 同步,
     * 此处仅清零积分以消除旧残差 (PID 旋转期间始终运行) */
    if (g_heading_locked) {
        g_heading_integral = 0.0f;
    }
}

/* ================================================================
 *  停 止
 * ================================================================
 */
void Chassis_Stop(void)
{
    /* 停止两个规划器 */
    TrapPlan_Stop(&g_trans_planner);
    TrapPlan_Stop(&g_rot_planner);

    /* 禁用航向PID, 防止 Stop 后 Update 覆盖电机速度 */
    g_heading_locked   = 0;
    g_heading_integral = 0.0f;

    /* 电机缓停 (slew limiter) */
    Stepper_SetSpeed(&g_motor_left,  0.0f);
    Stepper_SetSpeed(&g_motor_right, 0.0f);
}

/* ================================================================
 *  航 向 锁 定 开 关
 * ================================================================
 */
void Chassis_SetHeadingLock(uint8_t enable)
{
    g_heading_locked = (enable != 0) ? 1 : 0;
    g_heading_integral = 0.0f;
    if (g_heading_locked) {
        /* 快照当前传感器角度作为航向目标, 避免使用未定义的旧值
         * 后续 Moveto 不再覆写此目标, PID 持续维持锁定 */
        g_heading_target_rad = g_gyro_angle_z_rad;
    }
}

/* ================================================================
 *  传 感 器 注 入
 * ================================================================
 */
/* ---- 传感器注入 ---- */
void Chassis_SetGyroData(float gyro_angular_rad_s, float angle_z_rad)
{
    g_gyro_angular_rad_s = gyro_angular_rad_s;
    g_gyro_angle_z_rad   = angle_z_rad;
}

/* ---- 简化传感器注入 (信号量架构) ---- */
void chassis_feed_gyro(float angle_z_rad)
{
    g_gyro_angle_z_rad = angle_z_rad;
}

/* ---- 状态查询 ---- */
uint8_t Chassis_IsMoveDone(void)
{
    return TrapPlan_IsDone(&g_trans_planner);
}

uint8_t Chassis_IsRotateDone(void)
{
    return TrapPlan_IsDone(&g_rot_planner);
}

/* ================================================================
 *  统 一 管 线 更 新  (1ms)
 * ================================================================
 */
void Chassis_Update(void)
{
    float dt = (float)CONTROL_PERIOD_MS * 0.001f;  /* 0.001s */

    /* --------------------------------------------------------
     *  第1步: 平移规划器 → v (mm/s)
     * --------------------------------------------------------
     */
    float v_steps = 0.0f;
    if (!TrapPlan_IsDone(&g_trans_planner)) {
        v_steps = TrapPlan_Update(&g_trans_planner, dt);
    }
    /* v_steps 单位: steps/s → 转为 mm/s */
    float v_mm_s = STEPS2MM(v_steps);

    /* --------------------------------------------------------
     *  第2步: 旋转规划器 ★新 → omega_base (rad/s)
     * --------------------------------------------------------
     */
    float omega_base = 0.0f;
    if (!TrapPlan_IsDone(&g_rot_planner)) {
        omega_base = TrapPlan_Update(&g_rot_planner, dt);
    }
    /* omega_base 单位: rad/s (rot_planner 把虚拟 steps 当 rad 用) */

    /* --------------------------------------------------------
     *  第3步: 航向角度 PID → delta_omega (修正, ±15°/s)
     * --------------------------------------------------------
     */
    uint8_t rot_active = !TrapPlan_IsDone(&g_rot_planner);
    float delta_omega = 0.0f;
    if (g_heading_locked) {
        /* 旋转期间: 实时从梯形曲线插值位置同步 PID 目标, 闭环跟踪 */
        if (rot_active) {
            g_heading_target_rad = g_rot_planner.current_pos;
        }
        /* 旋转完成后: target 冻结在 final current_pos */

        float error = g_heading_target_rad - g_gyro_angle_z_rad;

        /* 死区: ±0.5° 过滤 MPU6050 零漂噪声 */
        if (fabsf(error) < 0.008726646f) {
            error = 0.0f;
            g_heading_integral = 0.0f;
        }

        /* PI 控制器 */
        g_heading_integral += error * dt;
        /* 积分 & 输出限幅 — 按场景切换 */
        float limit = rot_active ? HEADING_DELTA_LIMIT_RAD_S
                                 : HEADING_DELTA_LIMIT_NAV_RAD_S;
        float i_limit = limit / (HEADING_KI + 0.001f);
        g_heading_integral = CLAMP(g_heading_integral, -i_limit, i_limit);

        delta_omega = HEADING_KP * error + HEADING_KI * g_heading_integral;
        delta_omega = CLAMP(delta_omega, -limit, limit);
    }

    /* --------------------------------------------------------
     *  第4步: 串级融合 → 差速逆解
     *
     *    v_left  = v - omega_total * (base/2)
     *    v_right = v + omega_total * (base/2)
     *
     *  其中 omega_total = omega_base + delta_omega
     * --------------------------------------------------------
     */
    float omega_total = omega_base + delta_omega;
    float half_base_mm = WHEEL_BASE_MM * 0.5f;

    float v_left_mm_s  = v_mm_s - omega_total * half_base_mm;
    float v_right_mm_s = v_mm_s + omega_total * half_base_mm;

    /* 限幅到电机能力范围 */
    float max_wheel_speed_mm_s = (MOTOR_MAX_SPEED_RPM / 60.0f) * WHEEL_CIRCUMFERENCE_MM;
    v_left_mm_s  = CLAMP(v_left_mm_s,  -max_wheel_speed_mm_s, max_wheel_speed_mm_s);
    v_right_mm_s = CLAMP(v_right_mm_s, -max_wheel_speed_mm_s, max_wheel_speed_mm_s);

    /* --------------------------------------------------------
     *  第5步: 输出到电机层
     * --------------------------------------------------------
     */
    Stepper_SetWheelSpeed(&g_motor_left,  v_left_mm_s,
                          WHEEL_DIAMETER_MM, STEPS_PER_REV);
    Stepper_SetWheelSpeed(&g_motor_right, v_right_mm_s,
                          WHEEL_DIAMETER_MM, STEPS_PER_REV);
}