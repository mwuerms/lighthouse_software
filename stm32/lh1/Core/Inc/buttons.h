/**
 * Martin Egli
 * 2024-11-13
 * buttons for light house
 */

#ifndef _MM_BUTTONS_H_
#define _MM_BUTTONS_H_
 
// - use c standard libraries --------------------------------------------------
#include <stdint.h>

// - typedefs ------------------------------------------------------------------

// - public functions ----------------------------------------------------------

/**
 * nothing to do here, see void MX_GPIO_Init(void) in main.c
 */
void buttons_init(void);

void buttons_enable_irq(void);
void buttons_disable_irq(void);

#endif // _MM_BUTTONS_H_
