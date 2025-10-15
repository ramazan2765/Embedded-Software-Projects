
#ifndef DRIVERS_TMP102_H_
#define DRIVERS_TMP102_H_

#include "main.h"

#define TMP102_I2C_ADDR    (0x48<<1) // default address

HAL_StatusTypeDef TMP102_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature);

#endif /* DRIVERS_TMP102_H_ */
