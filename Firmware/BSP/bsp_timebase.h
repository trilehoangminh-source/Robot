#ifndef BSP_TIMEBASE_H
#define BSP_TIMEBASE_H

#include <stdint.h>

uint32_t bsp_timebase_now_ms(void);
/* Valid for elapsed intervals shorter than one full 2^32-ms wrap. */
uint32_t bsp_timebase_elapsed_ms(uint32_t start_ms, uint32_t end_ms);
uint32_t bsp_timebase_elapsed_since_ms(uint32_t start_ms);

#endif
