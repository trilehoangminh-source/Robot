#include "sg90.h"
#include "bsp_pwm.h"
static uint32_t pulses[2];
static bool configured,started;
bsp_status_t sg90_configure(uint32_t open,uint32_t close,bool verified) {
 if(!open || !close || open>=20000 || close>=20000)return BSP_INVALID_ARGUMENT;
 if(started)return BSP_BUSY;
 configured=false; if(!verified)return BSP_NOT_READY;
 pulses[0]=open; pulses[1]=close; configured=true; return BSP_OK;
}
bsp_status_t sg90_command(bool close) {
 bsp_status_t s;
 if(!configured)return BSP_NOT_READY;
 s=bsp_pwm_set_servo_pulse_us(pulses[close?1:0]); if(s!=BSP_OK)return s;
 if(!started) {s=bsp_pwm_start_servo(); if(s==BSP_OK)started=true;}
 return s; /* Command accepted, not measured mechanical position. */
}
bsp_status_t sg90_disable(void) {
 bsp_status_t s=bsp_pwm_stop_servo(); if(s==BSP_OK)started=false; return s;
}
