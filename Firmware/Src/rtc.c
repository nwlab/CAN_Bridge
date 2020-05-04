/*
 * rtc.c
 *
 *  Created on: May 3, 2020
 *      Author: nightworker
 */
#include <time.h>
#include "main.h"


void rtcGetTime(RTC_HandleTypeDef *hrtc, struct tm *timp)
{
  RTC_DateTypeDef rtcDate;
  RTC_TimeTypeDef rtcTime;
  HAL_RTC_GetTime(hrtc, &rtcTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(hrtc, &rtcDate, RTC_FORMAT_BIN);
  uint8_t hh = rtcTime.Hours;
  uint8_t mm = rtcTime.Minutes;
  uint8_t ss = rtcTime.Seconds;
  uint8_t d = rtcDate.Date;
  uint8_t m = rtcDate.Month;
  uint16_t y = rtcDate.Year;
  uint16_t yr = (uint16_t)(y+2000-1900);
  timp->tm_year = yr;
  timp->tm_mon = m - 1;
  timp->tm_mday = d;
  timp->tm_hour = hh;
  timp->tm_min = mm;
  timp->tm_sec = ss;
}
