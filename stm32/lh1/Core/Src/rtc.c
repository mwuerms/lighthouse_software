/*
 * rtc.c
 *
 *  Created on: Sep 24, 2025
 *      Author: martin
 */

#include "main.h"
#include "scheduler.h"
#include "rtc.h"
#include "leds.h"
#include "ui.h"

// - private functions ---------------------------------
static RTC_DateTypeDef present_date;
static RTC_TimeTypeDef present_time;

static inline void update_present_date_time(void) {
	// attention: MUST call HAL_RTC_GetTime() then call HAL_RTC_GetDate()
	HAL_RTC_GetTime(&hrtc, &present_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &present_date, RTC_FORMAT_BIN);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	update_present_date_time();

	scheduler_send_event(ui_tid, UI_EV_1S, NULL);
}

// - public functions -----------------------------------
const char rtc_weekday_names[][4] = {
	"---",
	"Mon",
	"Tue",
	"Wen",
	"Thu",
	"Fri",
	"Sat",
	"Sun",
};

void rtc_init(void) {
	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
}

void rtc_set_date_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t weekday) {
	RTC_DateTypeDef set_date = {.Year = year, .Month = month, .Date = day, .WeekDay = weekday};
	RTC_TimeTypeDef set_time = {.Hours = hour, .Minutes = min, .Seconds = sec};
	HAL_RTC_SetTime(&hrtc, &set_time, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &set_date, RTC_FORMAT_BIN);

	scheduler_send_event(ui_tid, UI_EV_TIME_UPDATE, NULL);
}

void rtc_get_date_time(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *weekday) {
	RTC_DateTypeDef get_date;
	RTC_TimeTypeDef get_time;
	HAL_RTC_GetTime(&hrtc, &get_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &get_date, RTC_FORMAT_BIN);
	*year = get_date.Year;
	*month = get_date.Month;
	*day = get_date.Date;
	*weekday = get_date.WeekDay;
	*hour = get_time.Hours;
	*min = get_time.Minutes;
	*sec = get_time.Seconds;
}

uint8_t rtc_get_present_hours(void) {
	return present_time.Hours;
}

uint8_t rtc_get_present_minutes(void) {
	return present_time.Minutes;
}

uint8_t rtc_get_present_seconds(void) {
	return present_time.Seconds;
}

uint8_t rtc_get_present_weekday(void) {
	return present_date.WeekDay;
}

void rtc_enable_1s_irq(void) {
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
}

void rtc_disable_1s_irq(void) {
	//HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
	HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
}
