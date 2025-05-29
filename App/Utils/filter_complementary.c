/**
 * @file filter_complementary.c
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "filter_complementary.h"
#include <math.h>

/**
 * @brief Complementary filter initialisation function
 * 
 * @param filter 
 * @param alpha 
 */
void ComplementaryFilter_Init(ComplementaryFilter_Data_t* filter, float alpha)
{
    filter->alpha = alpha;
    filter->attitude->roll = 0.0f;
    filter->attitude->pitch = 0.0f;
    filter->attitude->yaw = 0.0f;
}

/**
 * @brief Complemetary filter update function
 * 
 * @param filter 
 * @param imu_data 
 * @param dt 
 */
void ComplementaryFilter_Update(ComplementaryFilter_Data_t* filter, IMU_Data_t* imu_data, float dt)
{
    /* Attitude estimate from accelerometer values */
    float pitch_acc = atan2f(-imu_data->accel[0], sqrtf(imu_data->accel[1]*imu_data->accel[1] + imu_data->accel[2]*imu_data->accel[2]));
    float roll_acc = atan2f(imu_data->accel[1], imu_data->accel[2]);

    /* Integrate gyro values */
    float pitch_gyro = filter->attitude->pitch + imu_data->gyro[1] * dt;
    float roll_gyro = filter->attitude->roll + imu_data->gyro[0] * dt;

    /* Sensor fusion by complemetary filter */
    filter->attitude->pitch = filter->alpha * pitch_gyro + (1 - filter->alpha) * pitch_acc;
    filter->attitude->roll = filter->alpha * roll_gyro + (1 - filter->alpha) * roll_acc;

}

/**
 * @brief Function for reading attitude data result from complemetary filter
 * 
 * @param filter 
 * @return Attitude_Data_t 
 */
Attitude_Data_t* ComplementaryFilter_GetAttitude(ComplementaryFilter_Data_t* filter)
{
    return filter->attitude;
}