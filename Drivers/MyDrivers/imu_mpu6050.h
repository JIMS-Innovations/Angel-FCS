/**
 * @file imu_mpu6050.h
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #ifndef IMU_MPU6050
 #define IMU_MPU6050

 #include "sensor_types.h"
 #include "stm32f4xx_hal.h"

 bool MPU6050_Init(I2C_HandleTypeDef *hi2c);
 bool MPU6050_Read(IMU_Data_t *data);


 #endif/*IMU_MPU6050*/