
#include"packet.h"
#include<string.h>


extern UART_HandleTypeDef huart1;

void test_json_output(void)
{
	SensorPacket packet;
	char buffer[128];
	int len;

	// Fill the packet with some dummy values

	packet.temperature = 24.75f;
	packet.acceleration_x = 0.015f;
	packet.sound_level = 512;
	packet.humidity = 43;
	packet.timestamp_ms = HAL_GetTick();
	packet.valid = V_TEMP | V_ACCEL_X | V_SOUND | V_HUMIDITY;

   // Build JSON string
	len = packet_build_json(&packet, buffer, sizeof(buffer));

  // Send JSON over UART1
	HAL_UART_Transmit(&huart1,(uint8_t*)buffer, len, HAL_MAX_DELAY);

}
