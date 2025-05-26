/**
 * @file SensorTask.c
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "SensorTask.h"
#include "imu_mpu6050.h"
#include "main.h"
#include "i2c.h"

#define IMU_QUEUE_LENGTH 10

/* Task prototype */
void SensorTask_Run(void *arg);

/* Task Configuration */
osThreadDef(SensorTask, SensorTask_Run, osPriorityNormal, 0, 256);

/* Task Variables */
static osThreadId SensorTaskHandle;
static QueueHandle_t imuQueue;

void SensorTask_Init(void)
{
    /* Initialise MPU6050 */
    MPU6050_Init(&hi2c1);

    /* Create task */
    SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

    /* Create queue */
    imuQueue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(IMU_Data_t));
    
}

void SensorTask_Run(void *arg)
{
    IMU_Data_t imu;

    while (1)
    {
        if(MPU6050_Read(&imu))
        {
            imu.has_mag = false;
            imu.has_temp = true;

            xQueueSend(imuQueue, &imu, 0);
        }

        osDelay(10);
    }
    
}

bool SensorTask_GetIMU(IMU_Data_t *imu_out, TickType_t timeout)
{
    return(xQueueReceive(imuQueue, imu_out, timeout) == pdPASS);
}

