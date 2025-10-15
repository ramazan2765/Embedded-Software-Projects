#include "mpu9250.h"
#include <string.h>

// Registers
#define WHO_AM_I_REG      0x75
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B

HAL_StatusTypeDef MPU9250_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    // WHO_AM_I check (0x71 expected)
    if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, WHO_AM_I_REG, 1, &check, 1, 100) != HAL_OK)
        return HAL_ERROR;
    if (check != 0x71) return HAL_ERROR;

    // Wake up device (clear sleep bit)
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadAccelX(I2C_HandleTypeDef *hi2c, float *ax) {
    uint8_t rawData[2];
    int16_t rawAccelX;

    if (HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, ACCEL_XOUT_H, 1, rawData, 2, 100) != HAL_OK)
        return HAL_ERROR;

    rawAccelX = (int16_t)((rawData[0] << 8) | rawData[1]);
    *ax = ((float)rawAccelX) / 16384.0f;  // ±2g range

    return HAL_OK;
}
