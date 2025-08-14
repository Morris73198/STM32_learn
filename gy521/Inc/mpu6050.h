/* mpu6050.h */
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <math.h>

// MPU6050 I2C 地址
#define MPU6050_ADDR 0xD0  // 7位地址 0x68 左移1位

// MPU6050 暫存器地址
#define PWR_MGMT_1      0x6B  // 電源管理暫存器1
#define SMPLRT_DIV      0x19  // 採樣率分頻器
#define CONFIG          0x1A  // 配置暫存器
#define GYRO_CONFIG     0x1B  // 陀螺儀配置
#define ACCEL_CONFIG    0x1C  // 加速度計配置
#define WHO_AM_I        0x75  // 設備ID暫存器
#define ACCEL_XOUT_H    0x3B  // 加速度X軸高位元
#define GYRO_XOUT_H     0x43  // 陀螺儀X軸高位元
#define TEMP_OUT_H      0x41  // 溫度感測器高位元

// 函數宣告
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Test(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_Read_Temp(I2C_HandleTypeDef *hi2c, float *Temperature);
float MPU6050_Accel_To_G(int16_t accel_raw);
float MPU6050_Gyro_To_DPS(int16_t gyro_raw);
void MPU6050_Calculate_Angle(float accel_x, float accel_y, float accel_z, float *roll, float *pitch);

#endif /* INC_MPU6050_H_ */
