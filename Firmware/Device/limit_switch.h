#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H
#include <stdbool.h>
#include <stdint.h>
#include "bsp_status.h"
typedef struct { bool configured,active_high,initialized,raw,candidate,debounced,valid;
 uint32_t debounce_ms,changed_ms; } limit_switch_t;
bsp_status_t limit_switch_configure(uint8_t joint,bool active_high,uint32_t debounce_ms);
bsp_status_t limit_switch_filter(limit_switch_t *s,bool pin_high,uint32_t now_ms);
bsp_status_t limit_switch_read(uint8_t joint,uint32_t now_ms,limit_switch_t *state);
#endif
