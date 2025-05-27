/**
 * @file sensor_types.h
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct 
{
    float accel[3];     // X, Y, Z in m/s^2
    float gyro[3];      // X, Y, Z in deg/s
    float mag[3];       // X, Y, Z in µT (microtesla), optional
    float temperature;  // In °C (optional)

    bool has_mag;       // Flag to indicate if mag data is valid
    bool has_temp;      // Flag to indicate if temp data is valid

    uint32_t timestamp_ms;
} IMU_Data_t;

typedef struct 
{
    float pressure;       // in Pascals
    float altitude;       // in meters
    float temperature;    // °C

    bool has_temp;        // Is temperature valid?

    uint32_t timestamp_ms;
} Barometer_Data_t;


#endif/*SENSOR_TYPES_H*/