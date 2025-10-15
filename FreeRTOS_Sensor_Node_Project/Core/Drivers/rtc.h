
#ifndef DRIVERS_RTC_H_
#define DRIVERS_RTC_H_

#include "main.h"
#include <stdint.h>

// Zaman string buffer boyutu (örn. "2025-10-13 14:25:07")
#define RTC_TIMESTAMP_STR_SIZE   32

// RTC fonksiyonları
void RTC_Init(void);
void RTC_SetTimeAndDate(uint8_t year, uint8_t month, uint8_t date,
                        uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t weekday);
void RTC_GetTimestamp(char *buffer, uint16_t bufsize);


#endif /* DRIVERS_RTC_H_ */


