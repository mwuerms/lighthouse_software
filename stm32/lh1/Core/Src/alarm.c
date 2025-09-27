/*
 * alarm.c
 *
 *  Created on: Sep 27, 2025
 *      Author: martin
 */

#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "alarm.h"


static alarm_t alarms[ALARMS_SIZE];

void alarm_init(void) {
	return;
}

uint16_t alarm_get_alarm(uint16_t index, alarm_t *a) {
	if(index >= ALARMS_SIZE) {
		// error, invalid index
		return false;
	}
	memcpy(a, &alarms[index], sizeof(alarm_t));
	return true;
}
