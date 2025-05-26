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
static SemaphoreHandle_t imuMutex;
static IMU_Data_t imu_data;

void SensorTask_Init(void)
{
    /* Initialise MPU6050 */
    MPU6050_Init(&hi2c1);

    /* Create task */
    SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

    /* Create mutex */
    imuMutex = xSemaphoreCreateMutex();
    
}

void SensorTask_Run(void *arg)
{
    IMU_Data_t imu;

    while (1)
    {
        if(MPU6050_Read(&imu))
        {
            xSemaphoreTake(imuMutex, portMAX_DELAY);
            imu_data = imu;
            imu_data.has_mag = false;
            imu_data.has_temp = true;
            xSemaphoreGive(imuMutex);
        }

        osDelay(10);
    }
    
}

void SensorTask_GetIMU(IMU_Data_t *imu_out)
{
    if (xSemaphoreTake(imuMutex, portMAX_DELAY)) {
        *imu_out = imu_data;
        xSemaphoreGive(imuMutex);
    }
}

