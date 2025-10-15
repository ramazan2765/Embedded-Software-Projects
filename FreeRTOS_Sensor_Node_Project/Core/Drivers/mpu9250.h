#ifndef MPU9250_H
#define MPU9250_H

#include "main.h"

#define MPU9250_ADDR       0x68 << 1   // AD0=GND → 0x68

HAL_StatusTypeDef MPU9250_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU9250_ReadAccelX(I2C_HandleTypeDef *hi2c, float *ax);

#endif
