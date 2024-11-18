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

void leds_init(void);
void leds_front_dsiplay(char c);

void i2cLED_PowerUp(void);
void i2cLED_PowerDown(void);
void leds_front_display_time(uint8_t hour, uint8_t min, uint8_t colon, uint8_t days_mask, uint8_t pwm);

#endif /* INC_LEDS_H_ */
