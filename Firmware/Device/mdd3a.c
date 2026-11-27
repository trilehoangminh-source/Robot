#include "mdd3a.h"
#include <math.h>
#include "bsp_timebase.h"
bsp_status_t mdd3a_map(float u,float *a,float *b) {
 if (!a || !b || !isfinite(u)) return BSP_INVALID_ARGUMENT;
 if(u>1)u=1; if(u< -1)u=-1;
 *a=u>0?u:0; *b=u<0?-u:0; return BSP_OK;
}
static bsp_status_t fail(mdd3a_t *d,uint8_t j) {
 d->fault=true; d->enabled=false; (void)bsp_pwm_stop_motor(j); return BSP_IO_ERROR;
}
bsp_status_t mdd3a_enable(mdd3a_t *d,uint8_t j) {
 if(!d || j>=3)return BSP_INVALID_ARGUMENT;
 if(d->fault)return BSP_IO_ERROR; if(d->enabled)return BSP_BUSY;
 if(bsp_pwm_start_motor(j)!=BSP_OK)return fail(d,j);
 d->enabled=true; d->applied=0; d->reversing=false; return BSP_OK;
}
bsp_status_t mdd3a_disable(mdd3a_t *d,uint8_t j) {
 bsp_status_t s;
 if(!d || j>=3)return BSP_INVALID_ARGUMENT;
 s=bsp_pwm_stop_motor(j); d->enabled=false; d->applied=0; d->reversing=false;
 if(s!=BSP_OK)d->fault=true; return s;
}
bsp_status_t mdd3a_write(mdd3a_t *d,uint8_t j,float u) {
 float a,b;
 if(!d || j>=3 || mdd3a_map(u,&a,&b)!=BSP_OK)return BSP_INVALID_ARGUMENT;
 if(d->fault)return BSP_IO_ERROR; if(!d->enabled)return BSP_NOT_READY;
 /* Give preload registers at least one complete 20 kHz period at zero
  * before reversing. Caller retries; this does not block the task. */
 if(d->reversing) {
  if(bsp_timebase_elapsed_since_ms(d->reversal_ms)<1U)return BSP_BUSY;
  d->reversing=false;
 }
 if(d->applied*u<0) {
  if(bsp_pwm_set_motor_leg(j,BSP_PWM_LEG_A,0)!=BSP_OK ||
     bsp_pwm_set_motor_leg(j,BSP_PWM_LEG_B,0)!=BSP_OK)return fail(d,j);
  d->applied=0; d->reversal_ms=bsp_timebase_now_ms(); d->reversing=true;
  return BSP_BUSY;
 }
 /* Always remove the opposite leg first, including on reversal. PWM preload
  * takes effect on update; this is not a mechanical reversal ramp. */
 if(bsp_pwm_set_motor_leg(j,u>=0?BSP_PWM_LEG_B:BSP_PWM_LEG_A,0)!=BSP_OK ||
    bsp_pwm_set_motor_leg(j,u>=0?BSP_PWM_LEG_A:BSP_PWM_LEG_B,u>=0?a:b)!=BSP_OK)
    return fail(d,j);
 d->applied=a-b; return BSP_OK;
}
