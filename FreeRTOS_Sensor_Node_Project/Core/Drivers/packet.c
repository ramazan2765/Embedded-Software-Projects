#include"packet.h"
#include <stdio.h>

/**
 * @brief Serialize a SensorPacket into JSON format.
 *
 * This function converts only the valid fields (based on the bitmask)
 * into a JSON string representation. It ensures safe writing into the
 * provided buffer using snprintf to prevent overflow.
 *
 * Example output:
 *   {"temperature":24.62,"humidity":45}
 *
 * @param p               Pointer to the sensor packet
 * @param output_buffer   Destination buffer for the JSON string
 * @param outsize         Size of the output buffer
 * @return int            Number of characters written to the buffer
 */

int packet_build_json(const SensorPacket* p, char* output_buffer, int outsize)
{
    int number_of_character = 0;   // How many chars have been written
    int wrote_character = 0;       // Whether a field has already been written

    // Start JSON object
    number_of_character += snprintf(output_buffer + number_of_character,
                                    outsize - number_of_character,
                                    "{");

    // Temperature field
    if (p->valid & V_TEMP) {
        number_of_character += snprintf(output_buffer + number_of_character,
                                        outsize - number_of_character,
                                        "%s\"temperature\":%.2f",
                                        wrote_character ? "," : "",
                                        p->temperature);
        wrote_character = 1;
    }

    // Acceleration X field
    if (p->valid & V_ACCEL_X) {
        number_of_character += snprintf(output_buffer + number_of_character,
                                        outsize - number_of_character,
                                        "%s\"acceleration_x\":%.3f",
                                        wrote_character ? "," : "",
                                        p->acceleration_x);
        wrote_character = 1;
    }

    // Sound level field
    if (p->valid & V_SOUND) {
        number_of_character += snprintf(output_buffer + number_of_character,
                                        outsize - number_of_character,
                                        "%s\"sound_level\":%u",
                                        wrote_character ? "," : "",
                                        (unsigned)p->sound_level);
        wrote_character = 1;
    }

    // Humidity field
    if (p->valid & V_HUMIDITY) {
        number_of_character += snprintf(output_buffer + number_of_character,
                                        outsize - number_of_character,
                                        "%s\"humidity\":%u",
                                        wrote_character ? "," : "",
                                        (unsigned)p->humidity);
        wrote_character = 1;
    }



    // Timestamp field
    if (p->valid & V_TIME) {
        number_of_character += snprintf(output_buffer + number_of_character,
                                        outsize - number_of_character,
                                        "%s\"timestamp_ms\":%lu",
                                        wrote_character ? "," : "",
                                        (unsigned long)p->timestamp_ms);

        wrote_character = 1;
    }

    // Timestamp (RTC string)
    if (p->valid & V_TIME && p->timestamp_str[0] != '\0') {
        number_of_character += snprintf(output_buffer + number_of_character,
                                        outsize - number_of_character,
                                        "%s\"timestamp_date\":\"%s\"",
                                        wrote_character ? "," : "",
                                        p->timestamp_str);
        wrote_character = 1;
    }

    // Close JSON object
    number_of_character += snprintf(output_buffer + number_of_character,
                                    outsize - number_of_character,
                                    "}\n");

    return number_of_character;
}
