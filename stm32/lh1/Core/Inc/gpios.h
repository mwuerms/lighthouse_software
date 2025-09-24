/**
 * Martin Egli
 * 2024-11-13
 * gpios for light house
 */

#ifndef _MM_GPIOS_H_
#define _MM_GPIOS_H_
 
// - includes ------------------------------------------------------------------
#include <stdint.h>

// - typedefs ------------------------------------------------------------------

// - public functions ----------------------------------------------------------

/**
 * nothing to do here, see void MX_GPIO_Init(void) in main.c
 */
void gpios_init(void);

void gpios_button_enable_irq(void);
void gpios_button_disable_irq(void);
void gpios_vbus_enable_irq(void);
void gpios_vbus_disable_irq(void);

#endif // _MM_GPIOS_H_
