/**
 * @file    gray_sensor.c
 * @brief   八路灰度循迹模块 — 单次按需读取 & 车姿矫正 (亚博IRTack)
 * @note    基于 STM32F407VGT6 + HAL 库 + FreeRTOS
 *
 * 模块行为:
 *   上电后默认不发数据, 收到 $0,0,1# 后开始持续上报数字帧
 *   收到 $0,0,0# 后停止上报
 *
 * 本模块采用"开启→收帧→关闭"单次按需流程：
 *   车停稳 → 发开启指令 → 立即阻塞收帧 → 发关闭指令 → 模块静默
 *   (无 osDelay 等待, parse_frame 首字节超时 500ms 自然等待模块响应)
 *
 * 帧格式: $D,x1:<v>,x2:<v>,x3:<v>,x4:<v>,x5:<v>,x6:<v>,x7:<v>,x8:<v>#
 * 例: $D,x1:1,x2:0,x3:0,x4:1,x5:0,x6:0,x7:0,x8:0#
 *       x1 和 x4 检测到黑线
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
#include <stdlib.h>

/* 灰度读取成功标志 (freertos.c 陀螺仪任务轮询) */
volatile uint8_t g_gray_read_ok = 0;

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

/* ================================================================
 *  内 部 辅 助 函 数
 * ================================================================
 */

/**
 * @brief  阻塞读取 1 字节, 带超时
 * @param  pch      输出字节指针
 * @param  timeout  超时 (ms)
 * @return HAL_OK 成功, 其他为超时或错误
 */
static HAL_StatusTypeDef uart_read_byte(uint8_t *pch, uint32_t timeout)
{
    return HAL_UART_Receive(&huart6, pch, 1, timeout);
}

/**
 * @brief  清空 UART RX 缓冲区, 丢弃残留数据
 */
static void uart_flush_rx(void)
{
    uint8_t dummy;
    /* 快速清空: 以极短超时连续读取直到超时 */
    while (HAL_UART_Receive(&huart6, &dummy, 1, 1) == HAL_OK) {
        /* 丢弃 */
    }
}

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
 * @brief  解析一帧灰度传感器数据
 * @note   帧格式: $D,x1:v,x2:v,...,x8:v#
 *         逐字节状态机解析, 不依赖 sscanf (节省 flash)
 *         遇到 I 开头的版本字符串会自然跳过 (不是 $D 帧头)
 * @param  info  输出解析结果
 * @return 1=解析成功, 0=失败
 */
static uint8_t parse_frame(GraySensor_Info_t *info)
{
    uint8_t  ch;
    uint8_t  chan_idx = 0;       /* 当前通道索引 0~7 */
    uint8_t  state = 0;          /* 0=等'$', 1=等'D', 2=等',', 3=等'x', 4=等':', 5=等'0'/'1' */
    char     expect_x = 0;       /* 期望的 xN 编号, 1~8 */

    /* 超时已在外层处理, 此处仅逐字节解析 */
    for (;;) {
        /* 阻塞读 1 字节, 超时 500ms */
        if (uart_read_byte(&ch, GRAY_SENSOR_TIMEOUT_MS) != HAL_OK) {
            return 0;
        }

        switch (state) {
        case 0: /* 等待帧头 '$' */
            if (ch == '$') {
                state = 1;
            }
            /* 非 '$' 静默跳过 */
            break;

        case 1: /* 等待 'D' */
            if (ch == 'D') {
                state = 2;
            } else {
                state = 0;  /* 非法字符, 回退重新找帧头 */
                if (ch == '$') state = 1;  /* 但如果是 $ 就立即进入状态1 */
            }
            break;

        case 2: /* 等待 ',' 或直接以 x1 开头 */
            if (ch == ',') {
                state = 3;
                expect_x = 0;
            } else if (ch == 'x') {
                state = 4;
                expect_x = 0;
            } else if (ch == '#') {
                /* 空帧 (0通道), 帧尾提前 */
                state = 99;
            } else {
                /* 非法字符 */
                state = 0;
                if (ch == '$') state = 1;
            }
            break;

        case 3: /* 等待 'x' */
            if (ch == 'x') {
                state = 4;
                expect_x = 0;
            } else if (ch == '#') {
                state = 99;
            } else {
                state = 0;
                if (ch == '$') state = 1;
            }
            break;

        case 4: /* 等待通道编号数字 (1~8) */
            if (ch >= '1' && ch <= '8') {
                expect_x = ch - '0';       /* 1~8 */
                state = 5;
            } else {
                state = 0;
                if (ch == '$') state = 1;
            }
            break;

        case 5: /* 等待 ':' */
            if (ch == ':') {
                state = 6;
            } else {
                state = 0;
                if (ch == '$') state = 1;
            }
            break;

        case 6: /* 等待值 '0' 或 '1' */
            if (ch == '0' || ch == '1') {
                if (expect_x >= 1 && expect_x <= 8) {
                    uint8_t idx = expect_x - 1;
                    info->channels[idx] = (ch == '1') ? 1 : 0;
                    chan_idx++;
                }
                state = 2;  /* 准备下一个通道 (等 ',' 或 '#' 或 'x') */
            } else if (ch == '#') {
                /* 帧尾, 通道可能不足 */
                state = 99;
            } else {
                state = 0;
                if (ch == '$') state = 1;
            }
            break;

        case 99: /* 帧尾已遇到 */
        default:
            /* 帧尾, 跳出 */
            return (chan_idx == 8) ? 1 : 0;
        }

        /* 安全保护: 如果已解析8个通道, 等帧尾 */
        if (chan_idx >= 8) {
            /* 读取剩余直到 '#' */
            while (ch != '#') {
                if (uart_read_byte(&ch, GRAY_SENSOR_TIMEOUT_MS) != HAL_OK) {
                    return 0;
                }
            }
            return 1;
        }
    }
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
 * @brief  初始化灰度传感器串口
 * @note   USART6 已由 CubeMX (MX_USART6_UART_Init) 初始化,
 *         此处清空缓冲区保证干净态
 */
void GraySensor_Init(void)
{
    /* 确认 USART6 句柄有效 */
    if (huart6.Instance != USART6) {
        /* 异常: 强制调用 CubeMX 初始化保证可用 */
        MX_USART6_UART_Init();
    }

    /* 清空残留在 RX 缓冲区的数据 */
    uart_flush_rx();

    /* 确保模块初始为关闭态 (不发数据) */
    uart_send_cmd(CMD_STOP);
}

/**
 * @brief  单次按需读取灰度传感器数据
 * @note   完整流程:
 *           1. 发送 "$0,0,1#" 开启模块数字量上报
 *           2. 立即阻塞接收一帧, 状态机解析 (首字节超时 500ms, 无须 osDelay)
 *           3. 发送 "$0,0,0#" 关闭模块输出
 *           4. 清空残留数据 (关闭指令后可能还有最后一帧)
 *           5. 计算偏差, 返回结果
 *         调用前车必须已停稳
 * @return GraySensor_Info_t 解析结果, read_success 指示是否成功
 */
GraySensor_Info_t GraySensor_ReadOnce(void)
{
    GraySensor_Info_t info;
    memset(&info, 0, sizeof(info));

    /* 1. 清空旧数据, 确保干净态 */
    uart_flush_rx();

    /* 2. 发送开启指令 */
    if (uart_send_cmd(CMD_START) != HAL_OK) {
        info.read_success = 0;
        g_gray_read_ok = 2;  /* 发送失败, 通知调试打印任务 */
        return info;
    }

    /* 3. 立即阻塞接收一帧并解析 (parse_frame 首个字节超时500ms, 不丢帧) */
    if (parse_frame(&info)) {
        /* 计算偏差和辅助标志 */
        calc_offset(&info);
        info.read_success = 1;
        g_gray_read_ok = 1;  /* 读取成功, 通知调试打印任务 */
    } else {
        info.read_success = 0;
        g_gray_read_ok = 2;  /* 解析超时/失败, 通知调试打印任务 */
    }

    /* 5. 发送关闭指令, 模块停止上报 */
    uart_send_cmd(CMD_STOP);

    /* 6. 清空残留 (关闭指令后可能还有最后一帧在缓冲区) */
    uart_flush_rx();

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