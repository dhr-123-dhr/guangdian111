/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usart.h"
#include "gray_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "semphr.h"        // SemaphoreHandle_t, xSemaphoreCreateBinary, xSemaphoreTake
#include "mpu6050.h"       // MPU6050_Init, MPU6050_StartReadDMA, MPU6050_IntegrateYaw, g_mpu6050_data...
#include "chassis.h"       // Chassis_Init, Chassis_Moveto, Chassis_RotateTo, Chassis_IsMoveDone...

extern I2C_HandleTypeDef hi2c1;  // I2C1 句柄 (在 i2c.c 定义)



osThreadId_t gyroTaskHandle;
static const osThreadAttr_t gyroTask_attributes = {
    .name = "gyroTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};
static void StartGyroTask(void *argument);  // 前向声明

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile uint8_t g_chassis_initialized = 0;

/* 信号量驱动架构: I2C DMA 完成信号量 */
SemaphoreHandle_t g_i2c_dma_sem = NULL;

/* 1ms 控制定时器句柄 + 属性: 驱动梯形加减速状态机 */
osTimerId_t controlTimerHandle;
static const osTimerAttr_t controlTimer_attributes = {
    .name = "ControlTimer",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void ControlTimerCallback(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* ---- 1ms 控制定时器: 驱动梯形加减速状态机 ---- */
  controlTimerHandle = osTimerNew(ControlTimerCallback, osTimerPeriodic, NULL, &controlTimer_attributes);
  /* osTimerStart 移至 StartDefaultTask, 底盘初始化完成后再启动 */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* ---- 陀螺仪任务: 1ms DMA 读取 + 500ms 温度更新 ---- */
  gyroTaskHandle = osThreadNew(StartGyroTask, NULL, &gyroTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* ---- 初始化底盘 ---- */
  Chassis_Init();
  Chassis_SetHeadingLock(1);
  GraySensor_Init();  // 清空USART3缓冲区 + 确保模块静默
  g_chassis_initialized = 1;
  osTimerStart(controlTimerHandle, 1U);
  /* 等待陀螺仪初始化完成 (MPU6050_Init 约 1s) */
  osDelay(1500);

  /* ---- 任务状态机: 直行1m → 右转45° → 直行1m → 矫正 → 倒退1m → 停止 ---- */
  enum {
    STATE_FORWARD_1M,
    STATE_ROTATE_45,
    STATE_FORWARD_1M_2,
    STATE_GRAY_ROTATE_WAIT,
    STATE_REVERSE_1M,
    STATE_DONE
  } state = STATE_FORWARD_1M;
  uint8_t state_entered = 0;

  for (;;)
  {
    switch (state) {
      case STATE_FORWARD_1M:
        if (!state_entered) {
          Chassis_Moveto(1000.0f);
          state_entered = 1;
        }
        if (Chassis_IsMoveDone()) {
          state = STATE_ROTATE_45;
          state_entered = 0;
        }
        break;

      case STATE_ROTATE_45:
        if (!state_entered) {
          Chassis_RotateTo(-45.0f);
          state_entered = 1;
        }
        if (Chassis_IsRotateDone()) {
          state = STATE_FORWARD_1M_2;
          state_entered = 0;
        }
        break;

      case STATE_FORWARD_1M_2:
        if (!state_entered) {
          Chassis_Moveto(500.0f);
          state_entered = 1;
        }
        if (Chassis_IsMoveDone()) {
          /* 停稳后用灰度传感器矫正车姿 */
          osDelay(300);
          GraySensor_CorrectPose();
          state = STATE_GRAY_ROTATE_WAIT;
          state_entered = 0;
        }
        break;

      case STATE_GRAY_ROTATE_WAIT:
        if (Chassis_IsRotateDone()) {
          state = STATE_REVERSE_1M;
          state_entered = 0;
        }
        break;

      case STATE_REVERSE_1M:
        if (!state_entered) {
          Chassis_Moveto(-1000.0f);
          state_entered = 1;
        }
        if (Chassis_IsMoveDone()) {
          state = STATE_DONE;
          state_entered = 0;
        }
        break;

      case STATE_DONE:
        /* 任务完成, 永久等待 */
        break;
    }
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  陀螺仪任务: 信号量驱动 DMA 架构
 * @note   优先级高于 defaultTask, 保证传感器数据实时注入
 *         流程:
 *           1. MPU6050_StartReadDMA() 非阻塞启动传输
 *           2. xSemaphoreTake(g_i2c_dma_sem, 5ms) 等待 DMA 完成触发信号量
 *           3. DMA ISR → MPU6050_OnDMAComplete() 解析数据 + 释放信号量
 *           4. MPU6050_IntegrateYaw(dt) 积分角度
 *           5. chassis_feed_gyro(MPU6050_GetYaw()) 注入底盘
 *         超时保护: 5ms 无数据则继续循环, 不卡死
 */
void StartGyroTask(void *argument)
{
  (void)argument;

  /* 创建 DMA 完成信号量 (二进制, 初始不可用) */
  g_i2c_dma_sem = xSemaphoreCreateBinary();
  configASSERT(g_i2c_dma_sem != NULL);

  /* 初始化 MPU6050 (含零偏+温度标定, 约 1s) */
  MPU6050_Init(&hi2c1);

  uint32_t tick = 0;
  uint32_t print_cnt = 0;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);   /* 目标 1ms 周期 */

  for (;;) {
    /* 启动 DMA 异步读取 6 字节 (非阻塞, 立即返回) */
    MPU6050_StartReadDMA();

    /* 等待 DMA 完成信号量, 超时 5ms 防止 I2C 总线错误时永久阻塞 */
    BaseType_t sem_ret = xSemaphoreTake(g_i2c_dma_sem, pdMS_TO_TICKS(5));

    if (sem_ret == pdTRUE) {
      /* DMA 完成: 数据已由 ISR 中的 MPU6050_OnDMAComplete 解析 */
      /* 基于任务实际时间步长积分 */
      TickType_t now = xTaskGetTickCount();
      float dt = (float)(now - last_wake) * portTICK_PERIOD_MS * 0.001f;
      if (dt <= 0.0f) dt = 0.001f;   /* 最小 1ms */

      MPU6050_IntegrateYaw(dt);
      chassis_feed_gyro(MPU6050_GetYaw());

      /* 每 500ms 更新一次温度并重算温补零偏 */
      if (++tick >= 500) {
        tick = 0;
        MPU6050_UpdateTemperature(&hi2c1);
      }

      /* 每 100ms 通过 USART2 打印角速度与累积角度 (观测零漂) */
      if (++print_cnt >= 100) {
        print_cnt = 0;
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                          "ZRate:%+.3fd/s Yaw:%+.2fdeg\r\n",
                          g_mpu6050_data.gyro_z_dps,
                          g_mpu6050_data.angle_z_deg);
        if (len > 0 && len < (int)sizeof(buf)) {
          HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 10);
        }
        /* 灰度读取结果打印 */
        if (g_gray_read_ok == 1) {
          g_gray_read_ok = 0;
          char buf[80];
          int p = 0;
          for (int i = 0; i < 8; i++)
            p += snprintf(buf+p, sizeof(buf)-p, "%d ", g_gray_info_last.channels[i]);
          p += snprintf(buf+p, sizeof(buf)-p, "ofs=%.1f on=%d cross=%d\r\n",
                        g_gray_info_last.offset, g_gray_info_last.is_on_line, g_gray_info_last.is_crossroad);
          HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)p, 10);
        } else if (g_gray_read_ok == 2) {
          g_gray_read_ok = 0;
          HAL_UART_Transmit(&huart2, (uint8_t *)"huidu fail tx\r\n", 15, 10);
        } else if (g_gray_read_ok == 3) {
          g_gray_read_ok = 0;
          HAL_UART_Transmit(&huart2, (uint8_t *)"huidu fail rx\r\n", 15, 10);
        }
      }
    }
    /* 若超时: I2C 总线异常, 跳过本次采样, 继续循环不卡死 */

    /* 严格 1ms 周期补偿 (若提前返回则补齐, 保证采样率稳定) */
    vTaskDelayUntil(&last_wake, period);
  }
}

/**
 * @brief  1ms 控制定时器回调: 驱动梯形加减速状态机
 *         osTimerPeriodic 每 1ms 触发, 调用 Chassis_Update()
 */
static void ControlTimerCallback(void *argument)
{
    (void)argument;
    Chassis_Update();
}

/* USER CODE END Application */

