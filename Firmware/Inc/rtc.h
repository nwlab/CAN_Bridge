/*
 * rtc.h
 *
 *  Created on: May 3, 2020
 *      Author: nightworker
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "main.h"

extern RTC_HandleTypeDef hrtc;

void rtcGetTime(RTC_HandleTypeDef *hrtc, struct tm *timp);


#endif /* INC_RTC_H_ */
