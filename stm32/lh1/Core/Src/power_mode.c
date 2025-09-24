/**
 * Martin Egli
 * 2024-10-03
 * all needed functions to use power modes
 * coop scheduler for mcu
 * 
 * this file needs adaption to current mcu
 */

// - includes ------------------------------------------------------------------
//#define DEBUG_PRINTF_ON
#include "debug_printf.h"

#include "stm32l4xx_hal.h"

#include <string.h>
#include "power_mode.h"
#include "events.h"

// - private variables ---------------------------------------------------------
static uint8_t power_mode_cnt[NB_OF_POWER_MODES];
#define POWER_MODE_CNT_MAX (250)

// - private functions ---------------------------------------------------------
/**
 * get the deepest power mode
 */
static inline uint8_t get_deepest_power_mode(void) {
	if(power_mode_cnt[POWER_MODE_RUN])
		return POWER_MODE_RUN;
	if(power_mode_cnt[POWER_MODE_SLEEP])
		return POWER_MODE_SLEEP;
	return POWER_MODE_STOP; // deepest anyways
}

// - public functions ----------------------------------------------------------
void power_mode_init(void) {
	memset(power_mode_cnt, 0, sizeof(power_mode_cnt));
	// set to lowest power mode at the beginning
	power_mode_request(POWER_MODE_STOP);
}

void power_mode_request(uint8_t mode) {
	if(mode > POWER_MODE_STOP) {
		// unknown power mode
		return;
	}
	lock_interrupt(sr);
	if(power_mode_cnt[mode] < POWER_MODE_CNT_MAX) {
		power_mode_cnt[mode]++;
	}
	restore_interrupt(sr);
}

void power_mode_release(uint8_t mode) {
	if(mode > POWER_MODE_STOP) {
		// unknown power mode
		return;
	}
	lock_interrupt(sr);
	if(power_mode_cnt[mode]) {
		power_mode_cnt[mode]--;
	}
	restore_interrupt(sr);
}

void power_mode_sleep(void) {
	switch(get_deepest_power_mode()) {
		case POWER_MODE_RUN:
			// do not go to power, just idle here
			while (events_is_main_fifo_empty() == true);
			break;
		case POWER_MODE_SLEEP:
			// - mcu specific code here ------------
			while (events_is_main_fifo_empty() == true) {
				// stay here in sleep mode
				HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			}
			break;
		case POWER_MODE_STOP:
		default:
			// - mcu specific code here ------------
			while (events_is_main_fifo_empty() == true) {
				// stay here in stop mode
				HAL_SuspendTick();
				HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
				//HAL_ResumeTick();
				SystemClock_Config();
			}
			break;
	}
	return;
}
