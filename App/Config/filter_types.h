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

/**
 * @brief Attitude data structure
 * 
 */
typedef struct 
{
    float pitch;    // Pitch
    float roll;      // Roll
    float yaw;      // Yaw

    uint32_t timestamp_ms;
} Attitude_Data_t;

/**
 * @brief Complementary filter data structure
 * 
 */
typedef struct 
{
    float alpha;
    Attitude_Data_t* attitude;
    uint32_t sample_ms;

    uint32_t timestamp_ms;
} ComplementaryFilter_Data_t;
