#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_status.h"

/* Raw pin high/low on PE2, PE3, PE4. No polarity, debounce or homing policy. */
bsp_status_t bsp_gpio_read_limit_raw(uint8_t joint, bool *pin_high);

#endif
