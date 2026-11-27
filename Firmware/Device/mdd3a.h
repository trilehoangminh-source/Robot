#ifndef MDD3A_H
#define MDD3A_H
#include <stdbool.h>
#include "bsp_pwm.h"
typedef struct { bool enabled, fault; float applied; uint32_t reversal_ms; bool reversing; } mdd3a_t;
/* A/B=0/0 and 1/1 electrically brake; neither guarantees mechanical safety.
 * Cytron official MDD3A datasheet, Input Truth Table:
 * https://docs.google.com/document/d/1ax3gSo0srTzoSr2bo8ETLym0OqrYjlk_JCZK4kxtfXg/edit */
bsp_status_t mdd3a_map(float u, float *a, float *b);
bsp_status_t mdd3a_enable(mdd3a_t *d, uint8_t joint);
bsp_status_t mdd3a_disable(mdd3a_t *d, uint8_t joint);
bsp_status_t mdd3a_write(mdd3a_t *d, uint8_t joint, float u);
#endif
