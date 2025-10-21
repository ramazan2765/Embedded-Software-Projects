/*
 * packet.h
 *
 *  Created on: Oct 16, 2025
 *      Author: ramaz
 */

#ifndef DRIVERS_PACKET_H_
#define DRIVERS_PACKET_H_

#include <stdint.h>

/**
 * @brief Sensor data packet (MPU9250 + TMP102)
 */

typedef struct
{
	float accel[3];     /**< Accelerometer data: X, Y, Z (g) */
	float gyro[3];      /**< Gyroscope data: X, Y, Z (deg/s) */
	float temperature;  /**< Temperature from TMP102 (°C) */
	uint32_t timestamp; /**< System tick when data was sampled (ms) */
}sensorData_t;


/**
 * @brief Event types used in the system
 */

typedef enum
{
    EVENT_BUTTON_PRESS,   /**< User button pressed */
    EVENT_BUTTON_RELEASE, /**< User button released */
    EVENT_ERROR           /**< General error event */
} EventType_t;

typedef struct
{
	 	EventType_t type;   /**< Type of event */
	    uint32_t timestamp; /**< System tick when event occurred (ms) */
	    uint8_t data;       /**< Optional data (e.g., button count, error code) */
}Event_t;

#endif /* DRIVERS_PACKET_H_ */
