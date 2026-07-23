/**
 * @file    gray_sensor.c
 * @brief   八路灰度循迹模块 — DMA接收模式 (亚博IRTack)
 * @note    基于 STM32F407VGT6 + HAL 库 + FreeRTOS
 *
 * 通讯方式: USART6(PC6/PC7), 115200, DMA2 Stream1 Channel5 (RX Normal模式)
 *
 * 模块行为:
 *   上电后默认不发数据, 收到 $0,0,1# 后开始持续上报数字帧
 *   收到 $0,0,0# 后停止上报
 *
 * 本模块采用"开启→DMA收帧→关闭"单次按需流程：
 *   Start DMA → 发开启指令 → osDelay(50ms) 等DMA搬数据 → Stop DMA → 发关闭指令 → 解析
 *
 *   根因修复:
 *     轮询模式 HAL_UART_Receive 逐字节读, 115200下每字节87μs,
 *     HAL开销导致UART Overrun Error → 字节丢失 → 帧格式打碎。
 *     改为DMA接收: 硬件自动搬字节, CPU不参与, 不会overrun。
 *
 * 帧格式: $D,x1:<v>,x2:<v>,x3:<v>,x4:<v>,x5:<v>,x6:<v>,x7:<v>,x8:<v>#
 * 例: $D,x1:0,x2:1,x3:1,x4:0,x5:1,x6:1,x7:1,x8:1#
 *       x1 和 x4 检测到黑线 (0=黑线)
 *
 * 通道映射:
 *   x1 = L4 (最左, weight -4)   →   ch[0]
 *   x2 = L3 (左,   weight -3)   →   ch[1]
 *   x3 = L2 (中左, weight -2)   →   ch[2]
 *   x4 = L1 (微左, weight -1)   →   ch[3]
 *   x5 = R1 (微右, weight +1)   →   ch[4]
 *   x6 = R2 (中右, weight +2)   →   ch[5]
 *   x7 = R3 (右,   weight +3)   →   ch[6]
 *   x8 = R4 (最右, weight +4)   →   ch[7]
 */

#include "gray_sensor.h"
#include "usart.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* 灰度读取结果标志 (freertos.c 陀螺仪任务轮询)
 *   0 = 无事件
 *   1 = 读取成功
 *   2 = CMD_START 发送失败 或 DMA 启动失败
 *   3 = 解析失败 (超时或帧格式错)
 *   4 = 保留 (轮询模式旧错误码, 兼容)
 */
volatile uint8_t g_gray_read_ok = 0;
volatile GraySensor_Info_t g_gray_info_last = {0};

/* ================================================================
 *  内 部 常 量
 * ================================================================
 */

/* 8 通道权值: ch[0]=x1=L4(最左) ~ ch[7]=x8=R4(最右) */
static const int8_t s_weights[8] = {-4, -3, -2, -1, +1, +2, +3, +4};

/* 模块开启指令: 数字量上报 */
static const char *CMD_START = "$0,0,1#";
/* 模块关闭指令: 停止上报 */
static const char *CMD_STOP  = "$0,0,0#";
/* 指令长度 (含串结尾) */
#define CMD_LEN              7

/* DMA 接收缓冲区 (100字节 > 单帧43字节, 足够存一帧+额外数据) */
static uint8_t s_dma_buf[100];

/* ================================================================
 *  内 部 辅 助 函 数
 * ================================================================
 */

/**
 * @brief  发送指令到模块
 * @param  cmd  7字节指令字符串
 * @return HAL_OK 成功, 其他为错误
 */
static HAL_StatusTypeDef uart_send_cmd(const char *cmd)
{
    return HAL_UART_Transmit(&huart6, (uint8_t *)cmd, CMD_LEN, 100);
}

/**
 * @brief  从DMA缓冲区解析一帧灰度数据 (固定偏移取值)
 * @note   扫描 s_dma_buf 找帧头 '$D', 再找帧尾 '#'
 *         取值位置: s_dma_buf[frame_start + 6 + i*5], i=0~7
 *         与官方亚博代码 new_package[6+i*5] 完全一致
 * @param  info  输出解析结果
 * @return 1=解析成功, 0=失败
 */
static uint8_t parse_dma_frame(GraySensor_Info_t *info)
{
    /* 扫描找帧头 '$D' */
    int16_t frame_start = -1;
    for (uint8_t i = 0; i < 95; i++) {   /* 至少留5字节空间 */
        if (s_dma_buf[i] == '$' && s_dma_buf[i + 1] == 'D') {
            frame_start = i;
            break;
        }
    }
    if (frame_start < 0) {
        return 0;   /* 没找到帧头 */
    }

    /* 找帧尾 '#' */
    int16_t frame_end = -1;
    for (uint8_t i = frame_start; i < 100; i++) {
        if (s_dma_buf[i] == '#') {
            frame_end = i;
            break;
        }
    }
    if (frame_end < 0) {
        return 0;   /* 没找到帧尾 */
    }

    /* 固定偏移取值 (与官方亚博代码完全一致) */
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t val_pos = frame_start + 6 + i * 5;
        if (val_pos >= 100) {
            return 0;   /* 越界保护 */
        }
        char v = s_dma_buf[val_pos];
        if (v == '0') {
            info->channels[i] = 0;
        } else if (v == '1') {
            info->channels[i] = 1;
        } else {
            return 0;   /* 值不是0/1, 数据有问题 */
        }
    }

    return 1;
}

/**
 * @brief  计算偏差及其他辅助信息
 * @param  info  含 channels 的解析结果, 填充 offset/active_count/is_on_line/is_crossroad
 */
static void calc_offset(GraySensor_Info_t *info)
{
    float   sum_w = 0.0f;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (info->channels[i] == SENSOR_ACTIVE_LEVEL) {
            sum_w += (float)s_weights[i];
            count++;
        }
    }

    info->active_count = count;
    info->is_on_line   = (count >= 1) ? 1 : 0;
    info->is_crossroad = (count >= 5) ? 1 : 0;

    if (count > 0) {
        info->offset = sum_w / (float)count;
    } else {
        info->offset = 0.0f;
    }
}

/* ================================================================
 *  公 共 接 口
 * ================================================================
 */

/**
 * @brief  初始化灰度传感器 (确保模块静默)
 * @note   USART6 已由 CubeMX (MX_USART6_UART_Init) 初始化,
 *         此处仅发送关闭指令确保模块不发数据
 */
void GraySensor_Init(void)
{
    /* 确认 USART6 句柄有效 */
    if (huart6.Instance != USART6) {
        MX_USART6_UART_Init();
    }

    /* 确保模块初始为关闭态 (不发数据) */
    uart_send_cmd(CMD_STOP);

    /* 等待模块处理关闭指令 */
    osDelay(10);
}

/**
 * @brief  单次DMA接收灰度传感器数据并解析
 * @note   完整流程:
 *           1. 清零 info + DMA 缓冲区
 *           2. 启动 DMA 接收 (HAL_UART_Receive_DMA)
 *           3. 发送 "$0,0,1#" 开启模块数字量上报
 *           4. osDelay(50ms) 等待DMA搬完一帧数据
 *           5. 停 DMA 接收
 *           6. 发送 "$0,0,0#" 关闭模块输出
 *           7. 从 DMA 缓冲区解析帧数据
 *           8. 计算偏差, 返回结果
 *         调用前车必须已停稳
 * @return GraySensor_Info_t 解析结果, read_success 指示是否成功
 */
GraySensor_Info_t GraySensor_ReadOnce(void)
{
    GraySensor_Info_t info;
    memset(&info, 0, sizeof(info));
    memset(s_dma_buf, 0, sizeof(s_dma_buf));

    /* 1. 启动 DMA 接收 (Normal模式, 不会自动循环) */
    HAL_StatusTypeDef dma_ret = HAL_UART_Receive_DMA(&huart6, s_dma_buf, sizeof(s_dma_buf));
    if (dma_ret != HAL_OK) {
        info.read_success = 0;
        g_gray_read_ok = 2;   /* DMA 启动失败 */
        return info;
    }

    /* 2. 发送开启指令 */
    if (uart_send_cmd(CMD_START) != HAL_OK) {
        HAL_UART_DMAStop(&huart6);
        info.read_success = 0;
        g_gray_read_ok = 2;   /* 发送失败 */
        return info;
    }

    /* 3. 等待帧数据到达
     *    115200下1帧~43字节约4ms, 50ms等待非常宽裕
     *    模块收到指令后2~5ms开始发帧
     *    DMA 硬件自动搬字节进 s_dma_buf, CPU 让给其他任务 */
    osDelay(50);

    /* 4. 停 DMA + 关模块 */
    HAL_UART_DMAStop(&huart6);
    uart_send_cmd(CMD_STOP);

    /* 5. 解析 DMA 缓冲区中的帧数据 */
    if (parse_dma_frame(&info)) {
        calc_offset(&info);
        info.read_success = 1;
        g_gray_read_ok = 1;   /* 读取成功 */
    } else {
        info.read_success = 0;
        /* 诊断: 通过 USART2 打印 DMA 缓冲区前50字节 (遇到0x00就停) */
        char hex[205];
        int pos = 0;
        for (uint8_t i = 0; i < 50; i++) {
            if (s_dma_buf[i] == 0) break;
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", s_dma_buf[i]);
        }
        if (pos > 0) {
            snprintf(hex + pos, sizeof(hex) - pos, "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)"dma rx:", 7, 10);
            HAL_UART_Transmit(&huart2, (uint8_t *)hex, (uint16_t)strlen(hex), 100);
        }
        g_gray_read_ok = 3;   /* 解析失败 (超时或格式错) */
    }

    return info;
}

/**
 * @brief  一站式矫正 (非阻塞)
 * @note   内部: 读灰度 → 算偏差 → Chassis_RotateTo(angle) → 返回
 *         调用后状态机需检查 Chassis_IsRotateDone() 等待旋转完成
 *
 *         矫正逻辑:
 *           offset < 0 (偏左) → angle = offset * CORRECT_FACTOR
 *           offset > 0 (偏右) → angle = offset * CORRECT_FACTOR (负值=左转)
 *           脱线 (8路全0) 或路口 (≥5路) → 不矫正, 直接返回
 *
 *         使用 Chassis_RotateTo 而非直接操作电机:
 *           1ms 定时器的 Chassis_Update 持续运行航向 PID,
 *           统一管线接管避免两个指令源打架
 */
void GraySensor_CorrectPose(void)
{
    GraySensor_Info_t info = GraySensor_ReadOnce();
     
    /* 读取失败、脱线、路口: 不矫正 */
    if (!info.read_success || !info.is_on_line || info.is_crossroad) {
        return;
    }

    /* 偏移量极小 (abs < 0.3): 认为已居中, 跳过 */
    float abs_offset = (info.offset >= 0.0f) ? info.offset : -info.offset;
    if (abs_offset < 0.3f) {
        return;
    }

    /* 计算矫正角度: 偏差 × 系数 */
    float angle = -info.offset * CORRECT_FACTOR;

    /* 非阻塞: 下发旋转目标, 由 1ms 定时器统一管线接管 */
    Chassis_RotateTo(angle);
}