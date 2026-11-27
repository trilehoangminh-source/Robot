#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>
#include "bsp_status.h"

/* Pure modulo delta. Exactly half a counter range is direction-ambiguous. */
bsp_status_t bsp_encoder_delta(uint32_t previous, uint32_t current,
                                uint8_t counter_bits, int32_t *delta);

/* Explicitly start the CubeMX encoder timers. No count-to-angle conversion. */
bsp_status_t bsp_encoder_start(uint8_t joint);
bsp_status_t bsp_encoder_stop(uint8_t joint);
bsp_status_t bsp_encoder_read_raw(uint8_t joint, uint32_t *raw_count);
bsp_status_t bsp_encoder_sample_delta(uint8_t joint, int32_t *delta);
bsp_status_t bsp_encoder_resync(uint8_t joint);

#endif
