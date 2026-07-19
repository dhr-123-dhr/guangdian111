#ifndef __TRAP_PLAN_H__
#define __TRAP_PLAN_H__

#include <stdint.h>

/* ================================================================
 *  梯形加减速规划器 — 多实例、位置型、纯数学层
 *
 *  三参数模型: {max_speed, min_speed, max_accel}
 *  等效于 project/StepMotor.c 的 3 参数模型
 * ================================================================
 */

/* ---- 阶段枚举 ---- */
typedef enum {
    TRAP_IDLE,    /* 无目标, 静止                        */
    TRAP_ACCEL,   /* 加速段                              */
    TRAP_CRUISE,  /* 匀速段 (短距离三角形曲线无此段)      */
    TRAP_DECEL,   /* 减速段                              */
    TRAP_DONE     /* 已完成, 位置已钳位到目标             */
} TrapPhase_t;

/* ---- 规划器实例 ---- */
typedef struct {
    /* ---- 参数 ---- */
    float max_speed;    /* 最高速度  (steps/s)            */
    float min_speed;    /* 起步速度  (steps/s), 跳过死区   */
    float max_accel;    /* 最大加速度 (steps/s²)          */

    /* ---- 状态 ---- */
    float current_speed; /* 当前速度绝对值  (内部)        */
    float current_pos;   /* 当前绝对位置  (steps)        */
    float target_pos;    /* 目标绝对位置  (steps)        */

    TrapPhase_t phase;
    int8_t      direction; /* +1 或 -1                   */
} TrapezoidPlanner_t;

/* ================================================================
 *  API
 * ================================================================
 */

/**
 * @brief  初始化规划器实例
 * @param  tp         规划器指针
 * @param  max_speed  最高速度 (steps/s)
 * @param  min_speed  起步/最低速度 (steps/s), 跳过电机低速抖动区
 * @param  max_accel  最大加速度 (steps/s²)
 */
void TrapPlan_Init(TrapezoidPlanner_t *tp,
                   float max_speed, float min_speed, float max_accel);

/**
 * @brief  设置新的绝对位置目标 (可随时调用以切换目标)
 * @param  tp              规划器指针
 * @param  absolute_steps  目标绝对位置 (steps)
 *
 * @note  current_pos 不变, 若目标在当前位前方则为正方向运动;
 *        内部自动计算距离与方向, 重新初始化规划状态机。
 */
void TrapPlan_SetTarget(TrapezoidPlanner_t *tp, float absolute_steps);

/**
 * @brief  每周期调用, 更新速度与位置
 * @param  tp  规划器指针
 * @param  dt  时间步长 (秒), 通常 0.001
 * @return 当前期望速度 (steps/s, 带符号),
 *         到达目标后返回 0 且 phase = TRAP_DONE
 *
 * 核心公式:
 *   - 减速判据: remaining ≤ v²/(2a) → 切 DECEL
 *     (等价于 project 用 v² = v0² + 2a·x 判断)
 *   - 加速完美: 切 CRUISE 时 v ≥ max_speed
 *   - 短距离三角形: 加速中触发减速判据, 跳过 CRUISE
 */
float TrapPlan_Update(TrapezoidPlanner_t *tp, float dt);

/**
 * @brief  紧急停止 — 清除目标, 速度归零
 */
void TrapPlan_Stop(TrapezoidPlanner_t *tp);

/**
 * @brief  查询是否已完成 (IDLE 或 DONE)
 * @return 1=完成, 0=运行中
 */
uint8_t TrapPlan_IsDone(const TrapezoidPlanner_t *tp);

/**
 * @brief  获取当前绝对位置
 */
float TrapPlan_GetPosition(const TrapezoidPlanner_t *tp);

#endif /* __TRAP_PLAN_H__ */