/**
 * @file baro_mpl3115a2.h
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef BARO_MPL3115A2
#define BARO_MPL3115A2

#include "sensor_types.h"
#include "stm32f4xx_hal.h"

bool MPL3115A2_Init(I2C_HandleTypeDef *hi2c);
bool MPL3115A2_Read(Barometer_Data_t *data);

#endif/*BARO_MPL3115A2*/