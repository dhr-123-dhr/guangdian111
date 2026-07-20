#include "trap_plan.h"
#include <math.h>
#include <string.h>

/* ================================================================
 *  梯形加减速规划器 — 实现
 *
 *  三参数模型: {max_speed, min_speed, max_accel}
 *  核心公式: v² = v0² + 2·a·x
 *
 *  与 project/StepMotor.c 的 StepMotor_Pos_Run_Function 等价:
 *    - 减速判据: remaining ≤ v²/(2a)
 *      (project 等效: REAL_DEC_STEPS = v²/(2a), 步数计数器 ≥ POSSHIFT - DEC_STEPS)
 *    - 匀速判据: v ≥ max_speed
 *      (project 等效: ACC_STEPS 加速步数完成)
 *    - 短距离三角形: 加速中触发减速判据 → 跳过 CRUISE
 *      (project 等效: POSSHIFT < ACC_ADD_DEC_STEPS ⇒ ACC=DEC=POSSHIFT/2)
 *    - min_speed 起步: project 的 MinStartSpeed
 * ================================================================
 */

#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)<(b)?(a):(b))

/**
 * @brief  fabsf 封装
 */
static float my_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* ================================================================
 *  TrapPlan_Init
 * ================================================================
 */
void TrapPlan_Init(TrapezoidPlanner_t *tp,
                   float max_speed, float min_speed, float max_accel)
{
    memset(tp, 0, sizeof(TrapezoidPlanner_t));
    tp->max_speed = max_speed;
    tp->min_speed = min_speed;
    tp->max_accel = max_accel;
    tp->epsilon   = 0.5f;      /* steps 模式默认; rad 模式由调用者覆写为 ~0.0005 */
    tp->phase     = TRAP_IDLE;
}

/* ================================================================
 *  TrapPlan_SetTarget
 * ================================================================
 */
void TrapPlan_SetTarget(TrapezoidPlanner_t *tp, float absolute_steps)
{
    tp->target_pos = absolute_steps;
    float delta = absolute_steps - tp->current_pos;

    /* 已经在目标位置 — 直接完成 */
    if (my_abs(delta) < tp->epsilon) {
        tp->phase         = TRAP_DONE;
        tp->current_speed = 0.0f;
        tp->current_pos   = absolute_steps;
        tp->direction     = 1;
        return;
    }

    /* 确定方向 */
    tp->direction = (delta >= 0.0f) ? 1 : -1;

    /* 从 0 起步, 通过 ACCEL 段自然加速 (与电机层 slew limiter 保持同一起点)
     *
     * 极短行程检查: 若峰值速度 v_peak < min_speed,
     * 则跳过加速直接进入 DECEL, 由短距离三角形完成。
     */
    float total_dist   = my_abs(delta);
    float v_start      = 0.0f;

    /* 对称三角形峰值: v_peak² = 0² + a·total_dist → v_peak = sqrt(a·total_dist) */
    float v_peak_sq = tp->max_accel * total_dist;
    float v_peak    = sqrtf(MAX(v_peak_sq, 0.0f));

    /* 有效最高速度: 不超过 max_speed, 且短行程三角形不超过 v_peak */
    float eff_max_speed = MIN(tp->max_speed, v_peak);

    /* ---- 初始化 ACCEL 阶段 ---- */
    tp->current_speed = v_start;  /* = 0.0f */
    tp->phase         = TRAP_ACCEL;

    /* 若峰值速度 ≤ min_speed, 极短行程直接减速 */
    if (eff_max_speed <= tp->min_speed + 0.01f) {
        tp->phase = TRAP_DECEL;
    }
}

/* ================================================================
 *  TrapPlan_Update  —  核心状态机
 * ================================================================
 */
float TrapPlan_Update(TrapezoidPlanner_t *tp, float dt)
{
    if (tp->phase == TRAP_IDLE || tp->phase == TRAP_DONE) {
        /* 已停止, 返回 0 */
        return 0.0f;
    }

    float a      = tp->max_accel;
    float v      = tp->current_speed;
    float pos    = tp->current_pos;
    float target = tp->target_pos;
    int   dir    = tp->direction;

    /* 剩余距离 (标量, 始终 ≥ 0) */
    float remaining = my_abs(target - pos);

    switch (tp->phase) {

    /* ============================================================
     *  ACCEL — 加速段
     * ============================================================ */
    case TRAP_ACCEL: {
        /* 减速判据 (含 0.5·v·dt 离散补偿项, 防止过冲):
         *
         *  减速所需距离 = v²/(2a) + 0.5·v·dt
         *
         *  等价于 project: REAL_DEC_STEPS = v²/(2a),
         *  步数计数器 ≥ ABS_POSSHIFT - REAL_DEC_STEPS
         */
        float decel_dist = (v * v) / (2.0f * a) + 0.5f * v * dt;

        if (remaining <= decel_dist) {
            /* 进入减速段 — 短距离三角形曲线无匀速段 */
            tp->phase = TRAP_DECEL;
            goto decel_path;
        }

        /* 加速: v_new = v + a·dt, 限幅到 max_speed */
        float v_new = v + a * dt;
        if (v_new >= tp->max_speed - 0.01f) {
            /* 达到最高速 → 进入匀速段 */
            v_new = tp->max_speed;
            tp->phase = TRAP_CRUISE;
        }

        /* 积分位置 — 梯形法则: Δx = (v + v_new)/2 * dt */
        pos += (float)dir * (v + v_new) * 0.5f * dt;

        tp->current_speed = v_new;
        tp->current_pos   = pos;

        return (float)dir * v_new;
    }

    /* ============================================================
     *  CRUISE — 匀速段
     * ============================================================ */
    case TRAP_CRUISE: {
        /* 减速判据 */
        float decel_dist = (v * v) / (2.0f * a) + 0.5f * v * dt;

        if (remaining <= decel_dist) {
            tp->phase = TRAP_DECEL;
            goto decel_path;
        }

        /* 匀速 */
        pos += (float)dir * v * dt;
        tp->current_pos = pos;

        return (float)dir * v;
    }

    /* ============================================================
     *  DECEL — 减速段
     * ============================================================ */
    case TRAP_DECEL: decel_path: {
        /* 减速: v_new = v - a·dt */
        float v_new = v - a * dt;

        /* 减速到零或过零 → 终止, 钳位到目标 */
        if (v_new <= 0.0f) {
            tp->current_speed = 0.0f;
            tp->current_pos   = target;
            tp->phase         = TRAP_DONE;
            return 0.0f;
        }

        /* 积分位置 — 梯形法则: Δx = (v + v_new)/2 * dt */
        pos += (float)dir * (v + v_new) * 0.5f * dt;

        /* 到达或越过目标 — 钳位 */
        if (my_abs(target - pos) < tp->epsilon ||
            (dir > 0 && pos >= target) ||
            (dir < 0 && pos <= target))
        {
            tp->current_speed = 0.0f;
            tp->current_pos   = target;
            tp->phase         = TRAP_DONE;
            return 0.0f;
        }

        tp->current_speed = v_new;
        tp->current_pos   = pos;

        return (float)dir * v_new;
    }

    default:
        return 0.0f;
    }
}

/* ================================================================
 *  TrapPlan_Stop  —  紧急停止
 * ================================================================
 */
void TrapPlan_Stop(TrapezoidPlanner_t *tp)
{
    tp->phase         = TRAP_IDLE;
    tp->current_speed = 0.0f;
    tp->target_pos    = tp->current_pos;  /* 清除目标 */
}

/* ================================================================
 *  TrapPlan_IsDone
 * ================================================================
 */
uint8_t TrapPlan_IsDone(const TrapezoidPlanner_t *tp)
{
    return (tp->phase == TRAP_IDLE || tp->phase == TRAP_DONE) ? 1 : 0;
}

/* ================================================================
 *  TrapPlan_GetPosition
 * ================================================================
 */
float TrapPlan_GetPosition(const TrapezoidPlanner_t *tp)
{
    return tp->current_pos;
}