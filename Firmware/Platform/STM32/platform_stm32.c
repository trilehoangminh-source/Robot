#include "platform.h"
#include "bsp_encoder.h"
#include "encoder.h"
#include "bsp_timebase.h"
#include "motor.h"
#include "sg90.h"
#include "limit_switch.h"
static platform_status_t translate(bsp_status_t s) {
 switch(s) {
 case BSP_OK:return PLATFORM_OK;
 case BSP_INVALID_ARGUMENT:return PLATFORM_INVALID_ARGUMENT;
 case BSP_NOT_READY:return PLATFORM_NOT_READY;
 case BSP_BUSY:return PLATFORM_BUSY;
 default:return PLATFORM_IO_ERROR;
 }
}
static platform_status_t read_encoder(void *ctx,uint8_t j,uint32_t *raw) {
 (void)ctx; return translate(bsp_encoder_read_raw(j,raw));
}
static platform_status_t read_limit(void *ctx,uint8_t j,bool *active) {
 limit_switch_t state; bsp_status_t s; (void)ctx;
 s=limit_switch_read(j,bsp_timebase_now_ms(),&state);
 if(s==BSP_OK)*active=state.debounced; return translate(s);
}
static platform_status_t time_ms(void *ctx,uint32_t *now) {
 (void)ctx; *now=bsp_timebase_now_ms(); return PLATFORM_OK;
}
static platform_status_t set_motor(void *ctx,uint8_t j,float u) {
 (void)ctx; return translate(motor_write(j,u));
}
static platform_status_t set_gripper(void *ctx,platform_gripper_command_t command) {
 (void)ctx; return translate(sg90_command(command==PLATFORM_GRIPPER_CLOSE));
}
static platform_status_t feedback(void *ctx,uint8_t j,uint32_t step,uint32_t dt,platform_encoder_feedback_t *out){
 (void)ctx;return translate(encoder_sample(j,step,dt,out));
}
static platform_status_t cancel(void *ctx,uint8_t j){(void)ctx;return translate(motor_disable(j));}
static const platform_ops_t ops={read_encoder,read_limit,time_ms,set_motor,set_gripper,feedback,cancel};
platform_status_t platform_stm32_bind(platform_t *p) {
 if(!p)return PLATFORM_INVALID_ARGUMENT; p->ops=&ops; p->context=0; return PLATFORM_OK;
}
