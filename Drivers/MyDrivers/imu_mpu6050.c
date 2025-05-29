/**
 * @file imu_mpu6050.c
 * @author Jesutofunmi Kupoluyi (jimsufficiency@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-05-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #include "imu_mpu6050.h"

/* DEFINES */
#define MPU6050_ADDRESS       	(0X68 << 1) // IMU I2C address
#define WHO_AM_I				0X75        // Device ID
#define I2C_TIMEOUT 			1000        // 1000ms
// #define USE_GYRO_DEGS                       // if defined gyro output is in deg/s, else rad/s


/*  CONSTANTS   */
#define RAW_TO_G        4096 // For full scale range of +/- 8g
#define RAW_TO_MS2      0.00239420166f // GRAVITY/RAW_TO_G : 9.80665/4096 >> raw to ms/2
#define RAW_TO_DEGS_DIV 65.5 // For full scale range of +/- 500 deg/s
#define RAW_TO_DEGS     0.01526717557f // inv(RAW_TO_DEGS_DIV) : 1/65.5 >> raw to deg/s
#define RAW_TO_RADS     0.0002664624812f 

/* REGISTERS */
#define SELF_TEST_X   	0X0D // R/W
#define SELF_TEST_Y   	0X0E // R/w
#define SELF_TEST_Z   	0X0F // R/W
#define SELF_TEST_A   	0X10 // R/W
#define SMPLRT_DIV    	0X19 // R/W
#define CONFIG        	0X1A // R/W
#define GYRO_CONFIG   	0X1B // R/W
#define ACCEL_CONFIG  	0X1C // R/W
#define FIFO_EN       	0X23 // R/W
#define INT_PIN_CFG   	0X37 // R/W
#define INT_ENABLE    	0X38 // R/W
#define INT_STATUS    	0X3A // R/W
#define ACCEL_XOUT_H  	0X3B // R
#define ACCEL_XOUT_L  	0X3C // R
#define ACCEL_YOUT_H  	0X3D // R
#define ACCEL_YOUT_L  	0X3E // R
#define ACCEL_ZOUT_H  	0X3F // R
#define ACCEL_ZOUT_L  	0X40 // R
#define TEMP_OUT_H    	0X41 // R
#define TEMP_OUT_L    	0X42 // R
#define GYRO_XOUT_H   	0X43 // R
#define GYRO_XOUT_L   	0X44 // R
#define GYRO_YOUT_H   	0X45 // R
#define GYRO_YOUT_L   	0X46 // R
#define GYRO_ZOUT_H   	0X47 // R
#define GYRO_ZOUT_L   	0X48 // R
#define USER_CTRL     	0X6A // R/W
#define PWR_MGMT_1    	0X6B // R/W
#define PWR_MGMT_2    	0X6C // R/W



static I2C_HandleTypeDef *imu_i2c;
static uint8_t reg_data;
static HAL_StatusTypeDef status;
static uint8_t data_8[6];
static int16_t data_16[3];

/**
 * @brief MPU6050 initialisation function
 * 
 * @param hi2c 
 * @return true 
 * @return false 
 */
bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    imu_i2c = hi2c;

    /* Wake up IMU */
    reg_data = 0x00;
    status = HAL_I2C_Mem_Write(imu_i2c, MPU6050_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);

    /* Use on-chip LPF */
    reg_data = 0x02;
    status = HAL_I2C_Mem_Write(imu_i2c, MPU6050_ADDRESS, CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);

    /* Configuring gyro */
    reg_data = 0x08;
    status = HAL_I2C_Mem_Write(imu_i2c, MPU6050_ADDRESS, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);

    /* Configuring accelerometer */
    reg_data = 0x10;
    status = HAL_I2C_Mem_Write(imu_i2c, MPU6050_ADDRESS, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);

    return true;

}

/**
 * @brief MPU6050 read function
 * 
 * @param data 
 * @return true 
 * @return false 
 */
bool MPU6050_Read(IMU_Data_t *data)
{
    /* Read gyro data */
    status = HAL_I2C_Mem_Read(imu_i2c, MPU6050_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &data_8, 6, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);
    else
    {
        /* Combine raw values */
        data_16[0] = (data_8[0] << 8 | data_8[1]); // Angular velocity around X-axis (RAW)
        data_16[1] = (data_8[2] << 8 | data_8[3]); // Angular velocity around Y-axis (RAW)
        data_16[2] = (data_8[4] << 8 | data_8[5]); // Angular velocity around Z-axis (RAW)

#ifdef USE_GYRO_DEGS
        /* Convert values to deg/s */
        data->gyro[0] = RAW_TO_DEGS * data_16[0];
        data->gyro[1] = RAW_TO_DEGS * data_16[1];
        data->gyro[2] = RAW_TO_DEGS * data_16[2];
#else
        /* Convert values to rad/s and coordinates to NED */
        data->gyro[0] = RAW_TO_RADS * data_16[1];   // Y (RAW) now X (IMU)
        data->gyro[1] = RAW_TO_RADS * data_16[0];   // X (RAW) now Y (IMU)
        data->gyro[2] = -RAW_TO_RADS * data_16[2];  // Inverse Z (RAW) now Z (IMU)
#endif

    }
    
    /* Read accelerometer data */
    status = HAL_I2C_Mem_Read(imu_i2c, MPU6050_ADDRESS, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, &data_8, 6, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);
    else
    {
        /* Combine raw values */
        data_16[0] = (data_8[0] << 8 | data_8[1]);  // X-axis (RAW)
        data_16[1] = (data_8[2] << 8 | data_8[3]);  // Y-axis (RAW)
        data_16[2] = (data_8[4] << 8 | data_8[5]);  // Z-axis (RAW)

        /* Convert values to ms/2 and coordinates to NED */
        data->accel[0] = -RAW_TO_MS2 * data_16[1];  // Inverse Y (RAW) now X (IMU)
        data->accel[1] = -RAW_TO_MS2 * data_16[0];  // Inverse X (RAW) now Y (IMU)
        data->accel[2] = RAW_TO_MS2 * data_16[2];   // Z (RAW) now Z (IMU)
    }

    /* Read temperature data */
    status = HAL_I2C_Mem_Read(imu_i2c, MPU6050_ADDRESS, TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, &data_8, 2, I2C_TIMEOUT);
    if(status != HAL_OK) return(false);
    else
    {
        /* Combine raw values */
        data_16[0] = (data_8[0] << 8 | data_8[1]);

        /* Convert value to ˚C */
        data->temperature =  ((float) data_16[0] / 340) + 36.5;
    }

    return true;

}