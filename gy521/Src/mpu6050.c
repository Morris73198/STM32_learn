/* mpu6050.c */
#include "mpu6050.h"

// 初始化 MPU6050
void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // 1. 喚醒 MPU6050 (預設是睡眠模式)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);

    // 2. 設定採樣率分頻器 (1kHz / (1+7) = 125Hz)
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY);

    // 3. 設定低通濾波器 (5Hz 帶寬)
    data = 0x06;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    // 4. 設定陀螺儀量程 (±250°/s)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    // 5. 設定加速度計量程 (±2g)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
}

// 測試 MPU6050 連接是否正常
uint8_t MPU6050_Test(I2C_HandleTypeDef *hi2c) {
    uint8_t who_am_i;
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I, 1, &who_am_i, 1, HAL_MAX_DELAY);
    return (who_am_i == 0x68) ? 1 : 0;  // 應該回傳 0x68
}

// 讀取加速度計資料
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ) {
    uint8_t data[6];

    // 一次讀取6個位元組 (每軸2個位元組)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

    // 組合高低位元組 (大端序)
    *AccelX = (int16_t)(data[0] << 8 | data[1]);
    *AccelY = (int16_t)(data[2] << 8 | data[3]);
    *AccelZ = (int16_t)(data[4] << 8 | data[5]);
}

// 讀取陀螺儀資料
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ) {
    uint8_t data[6];

    // 一次讀取6個位元組
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

    // 組合高低位元組
    *GyroX = (int16_t)(data[0] << 8 | data[1]);
    *GyroY = (int16_t)(data[2] << 8 | data[3]);
    *GyroZ = (int16_t)(data[4] << 8 | data[5]);
}

// 讀取溫度感測器資料
void MPU6050_Read_Temp(I2C_HandleTypeDef *hi2c, float *Temperature) {
    uint8_t data[2];
    int16_t temp_raw;

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, TEMP_OUT_H, 1, data, 2, HAL_MAX_DELAY);
    temp_raw = (int16_t)(data[0] << 8 | data[1]);

    // 轉換公式：溫度 = (原始值/340.0) + 36.53
    *Temperature = (float)temp_raw / 340.0 + 36.53;
}

// 將原始加速度資料轉換為 g (重力加速度)
float MPU6050_Accel_To_G(int16_t accel_raw) {
    // ±2g 量程，16位元解析度：32768 / 2g = 16384 LSB/g
    return (float)accel_raw / 16384.0;
}

// 將原始陀螺儀資料轉換為度/秒
float MPU6050_Gyro_To_DPS(int16_t gyro_raw) {
    // ±250°/s 量程，16位元解析度：32768 / 250 = 131 LSB/(°/s)
    return (float)gyro_raw / 131.0;
}

// 計算傾斜角度 (Roll 和 Pitch)
void MPU6050_Calculate_Angle(float accel_x, float accel_y, float accel_z, float *roll, float *pitch) {
    // Roll: 繞X軸旋轉 (Y和Z軸平面的角度)
    *roll = atan2(accel_y, accel_z) * 180.0 / M_PI;

    // Pitch: 繞Y軸旋轉 (X和Z軸平面的角度)
    *pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0 / M_PI;
}
