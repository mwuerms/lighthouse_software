/*
 * alarm.h
 *
 *  Created on: Sep 27, 2025
 *      Author: martin
 */

#ifndef INC_ALARM_H_
#define INC_ALARM_H_

#include <stdint.h>

#define ALARMS_SIZE (8)
typedef struct {
	uint8_t state;
	struct {
		uint8_t hour, min, wd_mask;
	} alarm_time;
} alarm_t;

void alarm_init(void);
uint16_t alarm_get_alarm(uint16_t index, alarm_t *a);

#endif /* INC_ALARM_H_ */
