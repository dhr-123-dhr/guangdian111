#include "mpu6050.h"
#include "i2c.h"
#include "cmsis_os2.h"
#include <string.h>
#include <math.h>

/* ================================================================
 *  全 局 实 例
 * ================================================================
 */
MPU6050_Data_t g_mpu6050_data;
float g_gyro_z_bias_dps = 0.0f;

/* ================================================================
 *  内 部 缓 冲 & 校 准 变 量
 * ================================================================
 */
static uint8_t  g_rx_buf[6];                /* DMA 接收缓冲区: GYRO_XOUT_H ~ GYRO_ZOUT_L */
static uint8_t  g_temp_buf[7];              /* 阻塞读温度用的临时缓冲 */

/* 零偏校准结果 (静止 200 次均值, Init 时标定) */
static float    g_gyro_z_offset_at_ref = 0.0f;   /* 参考温度下的零偏 (°/s) */
static float    g_calib_temp_c = 25.0f;           /* 校准时的参考温度 (°C) */

/* 运行时温度缓存 (由 UpdateTemperature 更新) */
static float    g_current_temp_c = 25.0f;

/* ================================================================
 *  内 部 辅 助
 * ================================================================
 */

/**
 * @brief  阻塞写单字节寄存器 (校准阶段使用, 不冲突 DMA)
 */
static HAL_StatusTypeDef mpu_write_reg(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)(MPU6050_ADDR << 1),
                             reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/**
 * @brief  阻塞读取陀螺仪 Z 轴 2 字节 + 温度 2 字节 (校准阶段使用)
 * @param  hi2c    I2C 句柄
 * @param  raw_gyro_z  输出: 原始陀螺仪 Z 值
 * @param  raw_temp    输出: 原始温度值
 */
static HAL_StatusTypeDef mpu_read_sample(I2C_HandleTypeDef *hi2c,
                                         int16_t *raw_gyro_z,
                                         int16_t *raw_temp)
{
    uint8_t buf[4];
    HAL_StatusTypeDef ret;

    /* 读 GYRO_ZOUT_H + GYRO_ZOUT_L (0x47, 0x48) */
    ret = HAL_I2C_Mem_Read(hi2c, (uint16_t)(MPU6050_ADDR << 1),
                           MPU6050_REG_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT,
                           buf, 2, 100);
    if (ret != HAL_OK) return ret;

    *raw_gyro_z = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

    /* 读 TEMP_OUT_H + TEMP_OUT_L (0x41, 0x42) */
    ret = HAL_I2C_Mem_Read(hi2c, (uint16_t)(MPU6050_ADDR << 1),
                           MPU6050_REG_TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT,
                           buf, 2, 100);
    if (ret != HAL_OK) return ret;

    *raw_temp = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return HAL_OK;
}

/* ================================================================
 *  零 偏 + 温 度 标 定 (Init 时调用)
 * ================================================================
 */

/**
 * @brief  静止采样 200 次, 取 gyro_z 均值 + 温度均值
 * @param  hi2c  I2C 句柄
 * @note   采样间隔 ~5ms, 总耗时约 1s
 */
static void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c)
{
    float sum_gyro_z = 0.0f;
    float sum_temp   = 0.0f;
    int   valid      = 0;

    for (int i = 0; i < 200; i++) {
        int16_t raw_gyro_z, raw_temp;
        if (mpu_read_sample(hi2c, &raw_gyro_z, &raw_temp) == HAL_OK) {
            sum_gyro_z += (float)raw_gyro_z / MPU6050_GYRO_SENS_500;
            sum_temp   += ((float)raw_temp / MPU6050_TEMP_SCALE) + MPU6050_TEMP_OFFSET;
            valid++;
        }
        osDelay(5);   /* 5ms 间隔, 200次 ≈ 1s */
    }

    if (valid > 0) {
        g_gyro_z_offset_at_ref = sum_gyro_z / (float)valid;
        g_calib_temp_c         = sum_temp   / (float)valid;
    } else {
        /* 回退默认值 */
        g_gyro_z_offset_at_ref = 0.0f;
        g_calib_temp_c         = 25.0f;
    }

    /* 初始化运行时变量 */
    g_gyro_z_bias_dps  = g_gyro_z_offset_at_ref;
    g_current_temp_c   = g_calib_temp_c;
}

/* ================================================================
 *  公 共 接 口
 * ================================================================
 */

/**
 * @brief  初始化 MPU6050
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;

    /* 1. 唤醒 + 时钟源 = PLL_X */
    ret = mpu_write_reg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (ret != HAL_OK) return ret;
    osDelay(100);   /* 等待起振稳定 */

    /* 2. 采样率 = 1kHz (SMPLRT_DIV=0) — 配合 1ms 读取周期 */
    ret = mpu_write_reg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x00);
    if (ret != HAL_OK) return ret;

    /* 3. DLPF = 188Hz 带宽 (CONFIG=0x01) */
    ret = mpu_write_reg(hi2c, MPU6050_REG_CONFIG, 0x01);
    if (ret != HAL_OK) return ret;

    /* 4. 陀螺仪量程 = ±500°/s */
    ret = mpu_write_reg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x08);
    if (ret != HAL_OK) return ret;

    /* 5. 加速度计量程 = ±4g (可选, 本次未使用) */
    ret = mpu_write_reg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x08);
    if (ret != HAL_OK) return ret;

    /* 6. DLPF 切换后需等待稳定 */
    osDelay(50);

    /* 7. 零偏 + 温度标定 */
    MPU6050_Calibrate(hi2c);

    /* 8. 清零全局数据 */
    memset(&g_mpu6050_data, 0, sizeof(g_mpu6050_data));

    return HAL_OK;
}

/**
 * @brief  启动 DMA 异步读取陀螺仪 6 字节
 */
HAL_StatusTypeDef MPU6050_ReadGyro_DMA(I2C_HandleTypeDef *hi2c,
                                       MPU6050_Data_t *data)
{
    (void)data;   /* 数据由 ProcessData 在 DMA 回调中填入 g_mpu6050_data */

    return HAL_I2C_Mem_Read_DMA(hi2c,
                                (uint16_t)(MPU6050_ADDR << 1),
                                MPU6050_REG_GYRO_XOUT_H,
                                I2C_MEMADD_SIZE_8BIT,
                                g_rx_buf,
                                6);
}

/**
 * @brief  DMA 完成回调 — 换算 + 温度补偿 + 积分
 * @note   在中断上下文中调用, 锁最小化操作时间
 */
void MPU6050_ProcessData(MPU6050_Data_t *data)
{
    /* 拼合 6 字节 → 3 个 int16_t */
    int16_t raw_x = (int16_t)(((uint16_t)g_rx_buf[0] << 8) | g_rx_buf[1]);
    int16_t raw_y = (int16_t)(((uint16_t)g_rx_buf[2] << 8) | g_rx_buf[3]);
    int16_t raw_z = (int16_t)(((uint16_t)g_rx_buf[4] << 8) | g_rx_buf[5]);

    /* 换算为 °/s */
    data->gyro_x_dps = (float)raw_x / MPU6050_GYRO_SENS_500;
    data->gyro_y_dps = (float)raw_y / MPU6050_GYRO_SENS_500;
    data->gyro_z_dps = (float)raw_z / MPU6050_GYRO_SENS_500;

    /* 减去动态温度补偿零偏 */
    data->gyro_z_dps -= g_gyro_z_bias_dps;

    /* 换算 rad/s */
    data->gyro_z_rad_s = data->gyro_z_dps * 0.01745329252f;   /* × π/180 */

    /* Z 轴角度积分 (dt = 0.001s, 1ms 周期) */
    data->angle_z_rad += data->gyro_z_rad_s * 0.001f;
    data->angle_z_deg  = data->angle_z_rad * 57.2957795131f;   /* × 180/π */
}

/**
 * @brief  阻塞读取温度并更新动态零偏 (每 500ms 调用)
 */
void MPU6050_UpdateTemperature(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(hi2c, (uint16_t)(MPU6050_ADDR << 1),
                           MPU6050_REG_TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT,
                           buf, 2, 100);
    if (ret != HAL_OK) return;

    int16_t raw_temp = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    g_current_temp_c = ((float)raw_temp / MPU6050_TEMP_SCALE) + MPU6050_TEMP_OFFSET;

    /* 动态温补零偏: bias(T) = bias(T_ref) + coeff × (T - T_ref) */
    g_gyro_z_bias_dps = g_gyro_z_offset_at_ref
                      + MPU6050_GYRO_TEMP_COEFF * (g_current_temp_c - g_calib_temp_c);
}