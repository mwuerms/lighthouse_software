/**
 * Martin Wuerms
 * 2015-08-10
 */

/* - this file contains architecture specific definitions etc ... ----------- */

#ifndef _ARCH_H_
#define _ARCH_H_

/* - includes --------------------------------------------------------------- */
#include <stdint.h>
#include <cmsis_gcc.h>

/* - define ----------------------------------------------------------------- */
/* template to use later on
uint32_t primask;

primask = __get_PRIMASK();  // Save current interrupt state
__disable_irq();            // Disable all interrupts

// --- Critical Section Start ---
/ * Your critical code here * /
// --- Critical Section End ---

__set_PRIMASK(primask);     // Restore previous interrupt state
*/

// save status register + disable global interrupt
#define lock_interrupt(x) uint32_t x = __get_PRIMASK(); \
                          __disable_irq()

#define restore_interrupt(x) __set_PRIMASK(x)

/*
// save status register + disable global interrupt
#define lock_interrupt(x)
// restore status register again
#define restore_interrupt(x)
/ *
                            do { \
                            x = SR; \
                            DINT; \
                            } while(0)
*/

/* - typedef ---------------------------------------------------------------- */

/* - public functions ------------------------------------------------------- */


#endif /* _ARCH_H_ */
