/**
 * Martin Egli
 * 2024-03-29
 * module timestamp
 */

#ifndef _TIMESTAMP_h_
#define _TIMESTAMP_h_

#include <stdint.h>

void timestamp_start_counter(void);
void timestamp_stop_counter(void);
void timestamp_get_counter(uint32_t *now_s, uint32_t *now_ms);

#endif // _TIMESTAMP_h_
