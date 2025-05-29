/**
 * @file filter_complementary.h
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef FILTER_COMPLEMENTARY
#define FILTER_COMPLEMENTARY

#include "sensor_types.h"
#include "sensor_constants.h"
#include "filter_types.h"

void ComplementaryFilter_Init(ComplementaryFilter_Data_t* filter, float alpha);
void ComplementaryFilter_Update(ComplementaryFilter_Data_t* filter, IMU_Data_t* imu_data, float dt);
Attitude_Data_t* ComplementaryFilter_GetAttitude(ComplementaryFilter_Data_t* filter);

#endif/*FILTER_COMPLEMENTARY*/