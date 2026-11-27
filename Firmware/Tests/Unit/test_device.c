#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor.h"
#include "encoder.h"
#include "sg90.h"
#include "limit_switch.h"
#include "bsp_timebase.h"
static float legs[2]; static uint32_t tick; static int32_t step;
bsp_status_t bsp_pwm_set_motor_leg(uint8_t j,bsp_pwm_leg_t l,float u){(void)j;legs[l]=u;return BSP_OK;}
bsp_status_t bsp_pwm_start_motor(uint8_t j){(void)j;return BSP_OK;}
bsp_status_t bsp_pwm_stop_motor(uint8_t j){(void)j;return BSP_OK;}
bsp_status_t bsp_pwm_set_servo_pulse_us(uint32_t u){(void)u;return BSP_OK;}
bsp_status_t bsp_pwm_start_servo(void){return BSP_OK;}
bsp_status_t bsp_pwm_stop_servo(void){return BSP_OK;}
bsp_status_t bsp_encoder_start(uint8_t j){(void)j;return BSP_OK;}
bsp_status_t bsp_encoder_resync(uint8_t j){(void)j;return BSP_OK;}
bsp_status_t bsp_encoder_sample_delta(uint8_t j,int32_t *d){(void)j;*d=step;return BSP_OK;}
bsp_status_t bsp_gpio_read_limit_raw(uint8_t j,bool *r){(void)j;*r=true;return BSP_OK;}
bsp_status_t bsp_encoder_read_raw(uint8_t j,uint32_t *r){(void)j;*r=0;return BSP_OK;}
uint32_t bsp_timebase_now_ms(void){return tick;}
uint32_t bsp_timebase_elapsed_since_ms(uint32_t t){return tick-t;}
int main(void){
 float a,b,q,v; mdd3a_t d={0}; limit_switch_t sw={0};
 robot_joint_calibration_t c={0}; encoder_feedback_t feedback; platform_encoder_feedback_t f1,f2;
 assert(mdd3a_map(2,&a,&b)==BSP_OK && a==1 && b==0);
 assert(mdd3a_map(-0.5f,&a,&b)==BSP_OK && a==0 && b==0.5f);
 assert(mdd3a_map(NAN,&a,&b)==BSP_INVALID_ARGUMENT);
 assert(mdd3a_write(&d,0,1)==BSP_NOT_READY);
 assert(mdd3a_enable(&d,0)==BSP_OK);
 assert(mdd3a_write(&d,0,1)==BSP_OK);
 assert(mdd3a_write(&d,0,-1)==BSP_BUSY && legs[0]==0 && legs[1]==0);
 tick=1; assert(mdd3a_write(&d,0,-1)==BSP_OK && legs[1]==1);
 assert(motor_enable(0)==BSP_NOT_READY);
 assert(motor_configure(0,-1,0.4f,true)==BSP_OK);
 assert(motor_enable(0)==BSP_OK);
 assert(motor_write(0,2)==BSP_OK && legs[1]==0.4f);
 c.encoder_cpr=1000;c.encoder_sign=-1;c.calibration_valid=1;c.home_angle_rad=1;
 assert(encoder_convert(250,10,10,&c,&q,&v)==BSP_OK);
 assert(fabsf(q-(1-1.5707963f))<0.00001f && fabsf(v+6.2831853f)<0.00001f);
 assert(encoder_convert(0,0,0,&c,&q,&v)==BSP_INVALID_ARGUMENT);
 assert(encoder_configure(0,&c)==BSP_OK);step=10;
 assert(encoder_update(0,10,&feedback)==BSP_OK && feedback.valid && feedback.count==10);
 assert(encoder_sample(0,42,10,&f1)==BSP_OK);
 assert(encoder_sample(0,42,10,&f2)==BSP_OK&&f1.count==f2.count);
 c.encoder_cpr=0;assert(encoder_configure(0,&c)==BSP_INVALID_ARGUMENT);
 sw.configured=true;sw.active_high=false;sw.debounce_ms=5;
 assert(limit_switch_filter(&sw,false,UINT32_MAX-2)==BSP_NOT_READY);
 assert(limit_switch_filter(&sw,false,2)==BSP_OK && sw.debounced);
 assert(limit_switch_filter(&sw,true,3)==BSP_OK && sw.debounced);
 assert(limit_switch_filter(&sw,true,8)==BSP_OK && !sw.debounced);
 assert(sg90_command(false)==BSP_NOT_READY);
 assert(sg90_configure(0,2000,true)==BSP_INVALID_ARGUMENT);
 assert(sg90_configure(1000,2000,false)==BSP_NOT_READY);
 puts("device: PASS");return 0;
}
