/**
 * @file SensorTask.h
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

/* Includes */
#include "sensor_types.h"
#include "cmsis_os.h"

/* Public API */
void SensorTask_Init(void);
void SensorTask_GetIMU(IMU_Data_t *imu_out);

#endif/*SENSOR_TASK_H*/


