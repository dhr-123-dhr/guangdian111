#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx_hal.h"

/* ================================================================
 *  MPU6050 I2C 地址 (AD0=0, 7-bit)
 * ================================================================
 */
#define MPU6050_ADDR         0x68

/* ================================================================
 *  关 键 寄 存 器 定 义
 * ================================================================
 */
#define MPU6050_REG_SMPLRT_DIV   0x19   /* 采样率分频器 — 1kHz(0) / 125Hz(7) */
#define MPU6050_REG_CONFIG       0x1A   /* DLPF 带宽配置 */
#define MPU6050_REG_GYRO_CONFIG  0x1B   /* 陀螺仪量程 */
#define MPU6050_REG_ACCEL_CONFIG 0x1C   /* 加速度计量程 */
#define MPU6050_REG_PWR_MGMT_1   0x6B   /* 电源管理 — 唤醒 + 时钟源 */
#define MPU6050_REG_GYRO_XOUT_H  0x43   /* 陀螺仪 X 轴高字节 (连续 6 字节) */
#define MPU6050_REG_GYRO_YOUT_H  0x45
#define MPU6050_REG_GYRO_ZOUT_H  0x47
#define MPU6050_REG_TEMP_OUT_H   0x41   /* 温度传感器高字节 (连续 2 字节) */

/* ================================================================
 *  物 理 常 量
 * ================================================================
 */
#define MPU6050_GYRO_SENS_500         74.8f      /* ±500°/s 量程灵敏度: LSB/(°/s), 用法: dps = raw / 65.5 */
#define MPU6050_TEMP_OFFSET           36.53f   /* 温度偏移 (°C), 公式: °C = raw/340 + OFFSET */
#define MPU6050_TEMP_SCALE            340.0f   /* 温度比例因子 (LSB/°C) */

/* ================================================================
 *  温 度 补 偿 参 数 (可现场调参)
 * ================================================================
 */
#define MPU6050_GYRO_TEMP_COEFF       0.02f   /* 陀螺仪温漂系数 (°/s / °C), MPU6050 典型值 ~0.02~0.05, 需实测标定 */

/* ================================================================
 *  数 据 结 构
 * ================================================================
 */
typedef struct {
    float gyro_x_dps;         /* X 轴角速度 (°/s) */
    float gyro_y_dps;         /* Y 轴角速度 (°/s) */
    float gyro_z_dps;         /* Z 轴角速度 (°/s) — 底盘转向 */
    float gyro_z_rad_s;       /* Z 轴角速度 (rad/s) — 注入底盘 PID */
    float angle_z_deg;        /* Z 轴累积角度 (°) */
    float angle_z_rad;        /* Z 轴累积角度 (rad) */
} MPU6050_Data_t;

/* ================================================================
 *  全 局 实 例 & 动 态 零 偏
 * ================================================================
 */
extern MPU6050_Data_t g_mpu6050_data;

/* 运行时动态零偏 (°/s) — 已内置温度补偿, ProcessData 每次更新 */
extern float g_gyro_z_bias_dps;

/* ================================================================
 *  API 声 明
 * ================================================================
 */

/**
 * @brief  初始化 MPU6050 (唤醒 + 配置量程/采样率/DLPF + 零偏温度标定)
 * @param  hi2c  I2C1 句柄指针
 * @retval HAL_OK / HAL_ERROR
 * @note   内部调用 MPU6050_Calibrate: 静止采样 200 次取零偏 + 记录参考温度
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  启动 DMA 异步读取陀螺仪 6 字节 (GYRO_XOUT_H ~ GYRO_ZOUT_L)
 * @param  hi2c  I2C1 句柄
 * @param  data  输出数据结构体 (原始数据在 DMA 完成回调中填充)
 * @retval HAL_OK / HAL_ERROR
 * @note   函数立即返回, 数据在 HAL_I2C_MemRxCpltCallback 中由 MPU6050_ProcessData 处理
 */
HAL_StatusTypeDef MPU6050_ReadGyro_DMA(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

/**
 * @brief  DMA 读取完成回调 — 换算单位 + 温度补偿 + 积分角度
 * @param  data  包含已完成 DMA 填充的原始数据
 * @note   在 HAL_I2C_MemRxCpltCallback (中断上下文) 中调用, 不做耗时操作
 */
void MPU6050_ProcessData(MPU6050_Data_t *data);

/**
 * @brief  阻塞读取温度传感器并更新温补零偏 (每 500ms 调用一次)
 * @param  hi2c  I2C1 句柄
 * @note   每次调用更新 g_gyro_z_bias_dps = offset_at_ref + coeff × (T - T_ref)
 */
void MPU6050_UpdateTemperature(I2C_HandleTypeDef *hi2c);

/* ---- 信号量驱动架构新接口 ---- */

/**
 * @brief  上电静止零漂校准 (3σ鲁棒均值)
 * @param  samples  采集样本数 (建议 ≥ 200, 预热丢弃50个)
 * @note   调用前需确保MPU6050已正常输出, 底盘静止
 */
void MPU6050_CalibrateYaw(uint16_t samples);

/**
 * @brief  获取当前温度 (°C)
 * @retval 温度值
 */
float MPU6050_GetTempC(void);

/**
 * @brief  启动 DMA 异步读取陀螺仪 6 字节 (非阻塞)
 * @retval HAL_OK / HAL_ERROR
 * @note   函数立即返回, 完成后由 MPU6050_OnDMAComplete 在 ISR 中处理
 */
HAL_StatusTypeDef MPU6050_StartReadDMA(void);

/**
 * @brief  DMA 完成回调 — 解析原始数据 + 置完成标志 (ISR 中调用)
 * @note   在 HAL_I2C_MemRxCpltCallback 中调用, 锁最小化操作时间
 */
void MPU6050_OnDMAComplete(void);

/**
 * @brief  积分 Yaw 角度 (在任务上下文中调用, 唯一积分入口)
 * @param  dt  积分时间步长 (s)
 * @note   ProcessData 仅做 raw→物理量换算+零偏补偿, 不做积分; 本函数是唯一积分入口
 */
void MPU6050_IntegrateYaw(float dt);

/**
 * @brief  获取当前 Yaw 角度 (rad)
 * @retval 角度值 (rad)
 */
float MPU6050_GetYaw(void);

#endif /* __MPU6050_H__ */
