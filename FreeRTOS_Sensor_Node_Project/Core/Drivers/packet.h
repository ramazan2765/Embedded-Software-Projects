

#ifndef APP_PACKET_H_
#define APP_PACKET_H_

#include <stdint.h>
#include<main.h>

/**
 * @brief Unified structure to store all sensor values.
 * Each field may or may not be valid, depending on the sensor activity.
 * The 'valid' bitmask indicates which fields are currently filled.
 */

typedef struct {
  float    temperature;      // Temperature from TMP102 (°C)
  float    acceleration_x;   // Acceleration from MPU9250 (X-axis, g)
  uint16_t sound_level;      // Sound sensor raw value (ADC units)
  uint8_t  humidity;         // Relative humidity from DHT11 (%RH)
  uint32_t timestamp_ms;     // Timestamp in milliseconds
  char     timestamp_str[20]; /**< RTC ISO8601 string "YYYY-MM-DD HH:MM:SS" */
  uint8_t  valid;            // Bitmask indicating which fields are valid
} SensorPacket;

/* Bitmask flags for 'valid' field */
#define V_TEMP        (1u<<0)
#define V_ACCEL_X     (1u<<1)
#define V_SOUND       (1u<<2)
#define V_HUMIDITY    (1u<<3)
#define V_TIME		  (1u<<4)

/**
 * @brief Build a JSON string representation of the given SensorPacket.
 *
 * @param p      Pointer to the sensor packet
 * @param out    Output buffer for JSON string
 * @param outsz  Size of the output buffer
 * @return Number of characters written into 'out'
 */

packet_build_json(const SensorPacket* p, char* output_buffer, int outsize);

#endif /* APP_PACKET_H_ */
