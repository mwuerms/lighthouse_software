/*
 * ui.c
 *
 *  Created on: Sep 24, 2025
 *      Author: martin
 */

#include "main.h"
#include "scheduler.h"
#include "rtc.h"
#include "gpios.h"
#include "leds.h"
#include "ui.h"

static int8_t ui_task_func(uint8_t event, void *data);
static task_t ui_task = {.name = "UI", .task = ui_task_func};
int8_t ui_tid;

static uint8_t min_old = 60;
static int8_t ui_task_func(uint8_t event, void *data) {

	if(event == UI_EV_BUTTON0) {
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}
	if(event == UI_EV_BUTTON1) {
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	}
	if(event == UI_EV_BUTTON2) {
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	}
	if(event == UI_EV_BUTTON3) {
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}
	if(event == UI_EV_BUTTON4) {
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	}

	uint8_t hour = rtc_get_present_hours();
	uint8_t min  = rtc_get_present_minutes();
	uint8_t sec  = rtc_get_present_seconds();
	uint8_t wd   = rtc_get_present_weekday();
	uint8_t show_colon = sec & 0x01;

	if(event == UI_EV_1S) {
		if(min_old != min) {
			// update time
			scheduler_send_event(ui_tid, UI_EV_TIME_UPDATE, NULL);
		}
		else {
			// update colon only
			leds_front_display_update_colon(show_colon);
		}
		min_old = min;
	}
	if(event == UI_EV_TIME_UPDATE) {
		leds_front_display_update_time(hour, min, 1, leds_front_display_weekday_mask[wd]);
	}
	return 1; // stay on
}

void ui_init(void) {
	scheduler_add_task(&ui_task);
	ui_tid = ui_task.tid;
	scheduler_start_task(ui_tid);
}
