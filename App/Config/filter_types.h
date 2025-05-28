/**
 * @file filter_types.h
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct 
{
    float theta;    // Pitch
    float phi;      // Roll
    float psi;      // Yaw

    uint32_t timestamp_ms;
} Attitude_Data_t;