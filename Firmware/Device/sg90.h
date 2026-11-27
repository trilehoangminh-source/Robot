#ifndef SG90_H
#define SG90_H
#include <stdbool.h>
#include <stdint.h>
#include "bsp_status.h"
/* Configured pulses must be independently verified safe for the mechanism. */
bsp_status_t sg90_configure(uint32_t open_us,uint32_t close_us,bool verified);
bsp_status_t sg90_command(bool close);
bsp_status_t sg90_disable(void);
#endif
