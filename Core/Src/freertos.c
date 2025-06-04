/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "SensorTask.h"
#include "strings.h"
#include <stdio.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "minimal/mavlink.h"
#include "common/mavlink.h"
#include "common/common.h"

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
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_message_t heart_beat;
mavlink_message_t attitude_msg;
uint16_t len;

/* USER CODE END Variables */
osThreadId USBTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void USBTask_Run(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of USBTask */
  osThreadDef(USBTask, USBTask_Run, osPriorityHigh, 0, 512);
  USBTaskHandle = osThreadCreate(osThread(USBTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  SensorTask_Init();

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_USBTask_Run */
/**
  * @brief  Function implementing the USBTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_USBTask_Run */
void USBTask_Run(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN USBTask_Run */
  IMU_Data_t imu;
  Barometer_Data_t baro;
  Attitude_Data_t attitude;
  uint32_t last_ms = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
    SensorTask_GetIMU(&imu);
    SensorTask_GetBaro(&baro);
    SensorTask_GetAttitude(&attitude);
    
    char buffer[128];
    // snprintf(buffer, 128, "[IMU] Gyro(deg/s) >> X:%.2f, Y:%.2f, Z:%.2f | Acc(m/s2) >> X:%.2f, Y:%.2f, Z:%.2f | Baro >> %.2fm, %.2fPa, %.2f\r\n", imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.accel[0], imu.accel[1], imu.accel[2], baro.altitude, baro.pressure, baro.temperature);

    // snprintf(buffer, 128, "[IMU] Pitch: %.2f, Roll: %.2f\r\n[BARO] Altitude: %.2fm, Pressure: %.2fPa\r\n", RAD2DEG(attitude.pitch), RAD2DEG(attitude.roll), baro.altitude, baro.pressure);
    // CDC_Transmit_FS(buffer, strlen(buffer));

    if(HAL_GetTick() - last_ms >= 1000)
    {
      mavlink_msg_heartbeat_pack(1, 1, &heart_beat, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
      len = mavlink_msg_to_send_buffer(buf, &heart_beat);
      CDC_Transmit_FS(buf, len);
      last_ms = HAL_GetTick();
    }
    mavlink_msg_attitude_pack(1, 1, &attitude_msg, HAL_GetTick(), attitude.roll, attitude.pitch, 0, imu.gyro[1], imu.gyro[0], imu.gyro[2]);
    len = mavlink_msg_to_send_buffer(buf, &attitude_msg);
    CDC_Transmit_FS(buf, len);



    
    osDelay(50); // 20Hz

  }
  /* USER CODE END USBTask_Run */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
