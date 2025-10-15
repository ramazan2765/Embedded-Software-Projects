#include "rtc.h"
#include <stdio.h>

// RTC handle (CubeMX tarafından main.c’de oluşturulmalı)
extern RTC_HandleTypeDef hrtc;

/**
  * @brief  RTC Init wrapper (CubeMX zaten MX_RTC_Init oluşturuyor)
  */
void RTC_Init(void)
{
    // CubeMX'de MX_RTC_Init() fonksiyonunu çağırabilirsin
    MX_RTC_Init();
}

/**
  * @brief  Tarih ve saat ayarla (sadece ilk kurulumda çağır)
  * @param  year   : 00-99 (2000 + year)
  * @param  month  : 1-12
  * @param  date   : 1-31
  * @param  hours  : 0-23
  * @param  minutes: 0-59
  * @param  seconds: 0-59
  * @param  weekday: RTC_WEEKDAY_... (örn. RTC_WEEKDAY_MONDAY)
  */
void RTC_SetTimeAndDate(uint8_t year, uint8_t month, uint8_t date,
                        uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t weekday)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours   = hours;
    sTime.Minutes = minutes;
    sTime.Seconds = seconds;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

    sDate.WeekDay = weekday;
    sDate.Month   = month;
    sDate.Date    = date;
    sDate.Year    = year; // 2000 + year
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  RTC’den tarih-saat string üret
  * @param  buffer : çıktı string
  * @param  bufsize: buffer boyutu
  * @retval none
  */
void RTC_GetTimestamp(char *buffer, uint16_t bufsize)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    snprintf(buffer, bufsize,
             "%04d-%02d-%02d %02d:%02d:%02d",
             2000 + sDate.Year,
             sDate.Month,
             sDate.Date,
             sTime.Hours,
             sTime.Minutes,
             sTime.Seconds);
}
