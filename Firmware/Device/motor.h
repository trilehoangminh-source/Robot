#ifndef MOTOR_H
#define MOTOR_H
#include "mdd3a.h"
/* Configuration and enable are explicit; zero-initialized devices are disabled. */
bsp_status_t motor_configure(uint8_t joint,int sign,float max_output,bool verified);
bsp_status_t motor_enable(uint8_t joint);
bsp_status_t motor_disable(uint8_t joint);
bsp_status_t motor_write(uint8_t joint,float output);
#endif
