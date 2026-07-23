#ifndef __GRAY_SENSOR_H__
#define __GRAY_SENSOR_H__

#include "main.h"
#include <stdint.h>

/* ================================================================
 *  宏 定 义
 * ================================================================
 */

/* 黑线检测有效电平: 1=检测到黑线输出高 */
#define SENSOR_ACTIVE_LEVEL     0

/* 矫正角度系数 (degrees per offset_unit), 3°/单位偏差 */
#define CORRECT_FACTOR           3.0f

/* ================================================================
 *  数 据 结 构
 * ================================================================
 */

/**
 * @brief  灰度传感器单次读取结果
 */
typedef struct {
    uint8_t channels[8];   /* ch[0]=x1(L4最左) ~ ch[7]=x8(R4最右), 0=黑线 1=白 */
    float   offset;        /* 偏差值 -4.0 ~ +4.0 (负=偏左, 正=偏右, 0=居中) */
    uint8_t active_count;  /* 检测到黑线的通道数 */
    uint8_t is_on_line;    /* 是否检测到黑线 (≥1路触发) */
    uint8_t is_crossroad;  /* 是否路口 (≥5路同时触发) */
    uint8_t read_success;  /* 读取是否成功 (1=成功, 0=超时或解析失败) */
} GraySensor_Info_t;

/* 灰度读取成功标志 (被 freertos.c 陀螺仪任务轮询打印后清零) */
extern volatile uint8_t g_gray_read_ok;

/* 最近一次灰度读取结果 (供 freertos.c 打印详情) */
extern volatile GraySensor_Info_t g_gray_info_last;

/* ================================================================
 *  公 共 接 口
 * ================================================================
 */

/**
 * @brief  初始化灰度传感器串口
 * @note   USART3 已由 CubeMX (MX_USART3_UART_Init) 初始化,
 *         此函数做兼容占位, 可扩展额外初始化逻辑
 */
void GraySensor_Init(void);

/**
 * @brief  单次阻塞读取灰度传感器数据并解析
 * @note   车必须已停稳再调用, 使用 HAL_UART_Receive 逐字节接收,
 *         超时 500ms 保护
 * @return GraySensor_Info_t 解析结果
 */
GraySensor_Info_t GraySensor_ReadOnce(void);

/**
 * @brief  一站式矫正 (非阻塞)
 * @note   内部: 读灰度 → 算偏差 → Chassis_RotateTo(angle) → 返回
 *         调用后状态机需检查 Chassis_IsRotateDone() 等待旋转完成
 *         矫正逻辑:
 *           offset < 0 (偏左) → angle = offset * CORRECT_FACTOR, 正转右调
 *           offset > 0 (偏右) → angle = offset * CORRECT_FACTOR, 反转左调
 *           offset ≈ 0        → 居中, 跳过
 *         脱线 (8路全0) 或路口 (≥5路) 直接返回不矫正
 */
void GraySensor_CorrectPose(void);

#endif /* __GRAY_SENSOR_H__ */