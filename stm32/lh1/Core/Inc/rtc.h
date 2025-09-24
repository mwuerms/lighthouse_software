/*
 * rtc.h
 *
 *  Created on: Sep 24, 2025
 *      Author: martin
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include <stdint.h>

extern const char rtc_weekday_names[][4];
void rtc_init(void);

/**
 * set date and time
 * @param year    0 ... 99 (2000 ... 2099)
 * @param month   1 ... 12, use RTC_MONTH_JANUARY, ...
 * @param day     1 ... 31
 * @param hour    0 ... 23
 * @param min     0 ... 59
 * @param sec     0 ... 59
 * @param weekday 1 ... 7, use RTC_WEEKDAY_MONDAY, ...
 */
void rtc_set_date_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t weekday);
void rtc_get_date_time(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *weekday);
void rtc_enable_1s_irq(void);

#endif /* INC_RTC_H_ */
