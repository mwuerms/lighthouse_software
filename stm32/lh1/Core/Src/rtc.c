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

static int8_t rtc_task_func(uint8_t event, void *data);
static task_t rtc_task = {.name = "RTC", .task = rtc_task_func};
int8_t rtc_tid;
#define RTC_EV_1S (1)
#define RTC_EV_UPDATE_TIME (2)

// - private functions ---------------------------------
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	scheduler_send_event(rtc_tid, RTC_EV_1S, NULL);
}

static RTC_TimeTypeDef present_time;
static uint8_t rtc_old_minutes = 0;
static RTC_DateTypeDef present_date;

static int8_t rtc_task_func(uint8_t event, void *data) {
	uint8_t show_colon;

	HAL_RTC_GetTime(&hrtc, &present_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &present_date, RTC_FORMAT_BIN);

	if(present_time.Seconds & 0x01) {
		show_colon = 1;
	}
	else {
		show_colon = 0;
	}

	if(event == RTC_EV_1S) {
		if(rtc_old_minutes != present_time.Minutes) {
			// update time
			scheduler_send_event(rtc_tid, RTC_EV_UPDATE_TIME, NULL);
		}
		else {
			// update colon only
			leds_front_display_update_colon(show_colon);
		}
		rtc_old_minutes = present_time.Minutes;
	}
	if(event == RTC_EV_UPDATE_TIME) {
		leds_front_display_update_time(present_time.Hours, present_time.Minutes, 1, leds_front_display_weekday_mask[present_date.WeekDay]);
	}
	return 1; // stay on
}

// - public functions -----------------------------------

void rtc_init(void) {
	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);

	scheduler_add_task(&rtc_task);
	rtc_tid = rtc_task.tid;
	scheduler_start_task(rtc_tid);
}

void rtc_set_date_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t weekday) {
	RTC_DateTypeDef set_date = {.Year = year, .Month = month, .Date = day, .WeekDay = weekday};
	RTC_TimeTypeDef set_time = {.Hours = hour, .Minutes = min, .Seconds = sec};
	HAL_RTC_SetTime(&hrtc, &set_time, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &set_date, RTC_FORMAT_BIN);
	scheduler_send_event(rtc_tid, RTC_EV_UPDATE_TIME, NULL);
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
