
#include"tmp102.h"

HAL_StatusTypeDef TMP102_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature)
{
	uint8_t buffer[2];

	HAL_StatusTypeDef status;

	// TMP102 register 0x00 = temperature
	status = HAL_I2C_Mem_Read(hi2c, TMP102_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	if(status == HAL_OK){
		int16_t raw = (buffer[0] <<4 | (buffer[1] >>4));

	// Negative temperature check (12-bit two’s complement)
	if(raw & 0x800) {

		raw |= 0xF000;
	}

	*temperature = raw * 0.0625f;
	}

	return status;
}
