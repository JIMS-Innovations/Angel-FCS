/**
 * @file baro_mpl3115a2.c
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "baro_mpl3115a2.h"
#include "math.h"

/* Defines */
#define MPL3115A2_ADDRESS       (0xC0) // Barometer I2C address
#define I2C_TIMEOUT 			1000        // 1000ms
#define PRECISION               100         // 4 decimal places
#define SEA_LEVEL_PRESSURE      101326.0f
// #define ALTIMETER_MODE

/* Registers */
#define STATUS      0x00    // R
#define OUT_P_MSB   0x01    // R
#define OUT_P_CSB   0x02    // R
#define OUT_P_LSB   0x03    // R
#define OUT_T_MSB   0x04    // R
#define OUT_T_LSB   0x05    // R
#define WHO_AM_I    0x0C    // R
#define PT_DATA_CFG 0x13    // R/W
#define CTRL_REG1   0x26    // R/W



static I2C_HandleTypeDef *baro_i2c;
static uint8_t reg_data;
static HAL_StatusTypeDef status;
static uint8_t data_8[6];
static int16_t data_16;
static uint32_t data_u32;


bool MPL3115A2_Init(I2C_HandleTypeDef *hi2c)
{
    baro_i2c = hi2c;

    /* Set to standby mode */
    reg_data = 0xB8;
    status = HAL_I2C_Mem_Write(baro_i2c, MPL3115A2_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);

    /* Enable data flags */
    reg_data = 0x07;
    status = HAL_I2C_Mem_Write(baro_i2c, MPL3115A2_ADDRESS, PT_DATA_CFG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);

#ifdef ALTIMETER_MODE
    /* Set barometer active in altimeter mode, oversampling ratio: 64 @ 258ms */
    reg_data = 0xB1;
    status = HAL_I2C_Mem_Write(baro_i2c, MPL3115A2_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);
#else
    /* Set barometer active in barometer mode, oversampling ratio: 64 @ 258ms */
    reg_data = 0x31;
    status = HAL_I2C_Mem_Write(baro_i2c, MPL3115A2_ADDRESS, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);
#endif
    

    return true;
}

bool MPL3115A2_Read(Barometer_Data_t *data)
{
    
#ifdef ALTIMETER_MODE
    /* Read status register */
    status = HAL_I2C_Mem_Read(baro_i2c, MPL3115A2_ADDRESS, STATUS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);
   
    if((reg_data & 0x08))
    {
        /* Read pressure and temperature */
        status = HAL_I2C_Mem_Read(baro_i2c, MPL3115A2_ADDRESS, OUT_P_MSB, I2C_MEMADD_SIZE_8BIT, &data_8, 5, I2C_TIMEOUT);
        if(status != HAL_OK) return(false);
        else
        {
            data_16 = (data_8[0] << 8) | data_8[1];
            data->temperature = ((float)(data_8[3] * PRECISION) + (float)(data_8[4] >> 4)) / PRECISION;
            // data->temperature = (float)((data_8[4]>>4));
            data->altitude = (float)((data_16 * PRECISION) + (data_8[2] & 0xF0)) / PRECISION ;
        }
    }

#else

    /* Read status register */
    status = HAL_I2C_Mem_Read(baro_i2c, MPL3115A2_ADDRESS, STATUS, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);
   
    if((reg_data & 0x08))
    {
        /* Read pressure and temperature */
        status = HAL_I2C_Mem_Read(baro_i2c, MPL3115A2_ADDRESS, OUT_P_MSB, I2C_MEMADD_SIZE_8BIT, &data_8, 5, I2C_TIMEOUT);
        if(status != HAL_OK) return(false);
        else
        {
            data_u32 = (data_8[0] << 10) | (data_8[1] << 2) | (data_8[2] >> 6);
            data->temperature = (float)((data_8[3] * PRECISION) + data_8[4]) / PRECISION;
            data->pressure = (float)((data_u32 * PRECISION) + (float)(data_8[2] >> 4)) / PRECISION ;
            data->altitude = 44330.77 * (1 - pow((data->pressure/SEA_LEVEL_PRESSURE), 0.1902632));
        }
    }

#endif
    

    return true;
}