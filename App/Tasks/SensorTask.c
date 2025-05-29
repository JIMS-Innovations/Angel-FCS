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
#include "baro_mpl3115a2.h"
#include "main.h"
#include "i2c.h"



/* Task prototype */
void SensorTask_Run(void *arg);

/* Task Configuration */
osThreadDef(SensorTask, SensorTask_Run, osPriorityNormal, 0, 512);

/* Task Variables */
static osThreadId SensorTaskHandle;
static SemaphoreHandle_t imuMutex;
static SemaphoreHandle_t baroMutex;
static SemaphoreHandle_t attMutex;
static IMU_Data_t imu_data;
static Barometer_Data_t baro_data;
static Attitude_Data_t attitude;
static ComplementaryFilter_Data_t CFilter = {.attitude = &attitude};

void SensorTask_Init(void)
{
    /* Initialise MPU6050 */
    MPU6050_Init(&hi2c1);

    /* Initialise Barometer */
    MPL3115A2_Init(&hi2c1);

    /* Create task */
    SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

    /* Create mutex */
    imuMutex = xSemaphoreCreateMutex();
    attMutex = xSemaphoreCreateMutex();
    baroMutex = xSemaphoreCreateMutex();

    /* Initialise Complementary filter */
    ComplementaryFilter_Init(&CFilter, 0.98);
    
}

void SensorTask_Run(void *arg)
{
    IMU_Data_t imu;
    Barometer_Data_t baro;
    uint32_t last_ms = HAL_GetTick();
    float dt = 0.0f;

    while (1)
    {
        /* Calculate dt */
        uint32_t now_ms = HAL_GetTick();
        dt = (now_ms - last_ms) / 1000.0f;
        last_ms = now_ms;

        /* Read IMU */
        if(MPU6050_Read(&imu))
        {
            xSemaphoreTake(imuMutex, portMAX_DELAY);
            imu_data = imu;
            imu_data.has_mag = false;
            imu_data.has_temp = true;
            xSemaphoreGive(imuMutex);
        }

        /* Read Barometer */
        if(MPL3115A2_Read(&baro))
        {
            xSemaphoreTake(baroMutex, portMAX_DELAY);
            baro_data = baro;
            baro_data.has_temp = true;
            xSemaphoreGive(baroMutex);
        }

        /* Perform sensor fusion */
        ComplementaryFilter_Update(&CFilter, &imu, dt);


        osDelay(5);
    }
    
}

void SensorTask_GetIMU(IMU_Data_t *imu_out)
{
    if (xSemaphoreTake(imuMutex, portMAX_DELAY)) {
        *imu_out = imu_data;
        xSemaphoreGive(imuMutex);
    }
}

void SensorTask_GetAttitude(Attitude_Data_t *attitude_out)
{
    if (xSemaphoreTake(attMutex, portMAX_DELAY)) {
        *attitude_out = attitude;
        xSemaphoreGive(attMutex);
    }
}

void SensorTask_GetBaro(Barometer_Data_t *baro_out)
{
    if (xSemaphoreTake(baroMutex, portMAX_DELAY)) {
        *baro_out = baro_data;
        xSemaphoreGive(baroMutex);
    }
}

