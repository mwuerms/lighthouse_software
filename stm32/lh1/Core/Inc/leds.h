/**
 * Martin Egli
 * 2024-11-13
 * leds for light house
 */


#ifndef INC_LEDS_H_
#define INC_LEDS_H_

// - includes ------------------------------------------------------------------
#include <stdint.h>

// - typedefs ------------------------------------------------------------------

// - public functions ----------------------------------------------------------
extern uint8_t leds_front_display_weekday_mask[8];

void leds_init(void);
void leds_front_dsiplay(char c);

void leds_front_dsiplay_power_up(void);
void leds_front_dsiplay_power_down(void);
void leds_front_dsiplay_set_brightness(uint8_t pwm);
void leds_front_display_update_time(uint8_t hour, uint8_t min, uint8_t colon, uint8_t days_mask);
void leds_front_display_update_colon(uint8_t colon);

#endif /* INC_LEDS_H_ */
