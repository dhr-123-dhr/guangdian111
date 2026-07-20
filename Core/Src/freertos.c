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
#include "timers.h"
#include "main.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "mpu6050.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* 1ms 控制定时器句柄 */
osTimerId_t controlTimerHandle;
const osTimerAttr_t controlTimer_attributes = {
  .name = "ControlTimer"
};

/* 陀螺仪任务句柄 */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* 1ms 控制定时器回调前向声明 (定义在下方) */
static void ControlTimerCallback(void *argument);
void StartGyroTask(void *argument);


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
  osTimerStart(controlTimerHandle, 1U);  /* 1ms 周期 */

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
/* 1ms 控制定时器回调: 更新梯形加减速状态机 */
void ControlTimerCallback(void *argument)
{
  Chassis_Update();
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* ---- 初始化底盘 ---- */
  Chassis_Init();
  Chassis_SetHeadingLock(1);
  /* 等待陀螺仪初始化完成 (MPU6050_Init 约 1s) */
  osDelay(1500);

  for (;;)
  {
    
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  陀螺仪任务: 1ms 周期 DMA 读取 + 500ms 温度更新
 * @note   优先级高于 defaultTask, 保证传感器数据实时注入
 */
void StartGyroTask(void *argument)
{
  (void)argument;

  /* 初始化 MPU6050 (含零偏+温度标定, 约 1s) */
  MPU6050_Init(&hi2c1);

  uint32_t tick = 0;

  for (;;) {
    /* 启动 DMA 异步读取陀螺仪 6 字节 */
    MPU6050_ReadGyro_DMA(&hi2c1, &g_mpu6050_data);
    osDelay(1);  /* 1ms 周期 — DMA 在 540μs 内完成, 回调在 delay 期间触发 */

    /* 注入陀螺仪数据到底盘 (角速度 + 累积角度) */
    Chassis_SetGyroData(g_mpu6050_data.gyro_z_rad_s, g_mpu6050_data.angle_z_rad);

    /* 每 500ms 更新一次温度并重算温补零偏 */
    if (++tick >= 500) {
      tick = 0;
      MPU6050_UpdateTemperature(&hi2c1);
    }
  }
}

/* USER CODE END Application */