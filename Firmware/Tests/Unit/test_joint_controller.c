#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "joint_controller.h"
static uint32_t raw[3];static unsigned writes;static platform_status_t read_status=PLATFORM_OK,write_status=PLATFORM_OK;
static platform_status_t read_count(void *ctx,uint8_t id,uint32_t *out){(void)ctx;*out=raw[id];return read_status;}
static platform_status_t write_motor(void *ctx,uint8_t id,float u){(void)ctx;(void)id;(void)u;++writes;return write_status;}
static platform_status_t converted(void *ctx,uint8_t id,uint32_t step,uint32_t dt,platform_encoder_feedback_t *f){
 (void)ctx;(void)step;(void)dt;f->raw=raw[id];f->count=raw[id];f->q=(float)raw[id]*0.0062831853f;f->w=0;f->valid=true;f->reference_epoch=1;return read_status;
}
int main(void){
 const platform_ops_t ops={read_count,0,0,write_motor,0,converted,0};platform_t p={&ops,0};
 joint_controller_t j[3];joint_config_t c={0};joint_feedback_t f;joint_diagnostics_t d;unsigned i;
 c.calibration.encoder_cpr=1000;c.calibration.encoder_sign=1;c.calibration.calibration_valid=1;
 c.calibration.limits_valid=1;c.calibration.min_angle_rad=-10;c.calibration.max_angle_rad=10;
 c.pid.kp=1;c.pid.ki=1;c.pid.dt=0.001f;c.pid.out_min=-0.3f;c.pid.out_max=0.3f;c.pid.i_min=-0.1f;c.pid.i_max=0.1f;
 c.kalman.sa=0.1f;c.kalman.r=0.001f;c.kalman.p00=1;c.kalman.p11=1;c.parameters_verified=true;c.busy_timeout_ms=5;
 for(i=0;i<3;i++){
 assert(joint_controller_initialize(&j[i],(uint8_t)i,&p)==JOINT_OK);
 assert(joint_controller_enable(&j[i])==JOINT_NOT_READY);
 assert(joint_controller_configure(&j[i],&c)==JOINT_OK);
 assert(joint_controller_enable(&j[i])==JOINT_NOT_READY);
 assert(joint_controller_update(&j[i])==JOINT_OK);
 assert(joint_controller_set_reference(&j[i],0)==JOINT_OK);
 assert(joint_controller_enable(&j[i])==JOINT_OK);
 }
 assert(writes==0);
 assert(joint_controller_set_reference(&j[0],1)==JOINT_OK);
 assert(joint_controller_update(&j[0])==JOINT_OK&&j[0].diagnostics.requested_output==0.3f&&j[0].pid.integral==0);
 assert(joint_controller_set_reference(&j[1],-1)==JOINT_OK);
 assert(joint_controller_update(&j[1])==JOINT_OK&&j[1].diagnostics.requested_output==-0.3f);
 assert(j[2].pid.integral==0&&j[2].diagnostics.reference==0);
 raw[0]=1;assert(joint_controller_update(&j[0])==JOINT_OK);
 assert(joint_controller_get_feedback(&j[0],&f)==JOINT_OK&&f.measured_q>0&&f.valid);
 assert(joint_controller_get_diagnostics(&j[0],&d)==JOINT_OK&&!d.applied_output_known&&d.write_accepted);
 assert(joint_controller_set_reference(&j[0],NAN)==JOINT_INVALID);
 assert(joint_controller_set_reference(&j[0],INFINITY)==JOINT_INVALID);
 write_status=PLATFORM_BUSY;
 assert(joint_controller_update(&j[0])==JOINT_NOT_READY&&!j[0].fault);
 assert(j[0].pid.integral==0);
 write_status=PLATFORM_IO_ERROR;
 assert(joint_controller_update(&j[0])==JOINT_FAULT&&!j[0].enabled);
 write_status=PLATFORM_OK;assert(joint_controller_enable(&j[0])==JOINT_FAULT);
 assert(joint_controller_reset(&j[0])==JOINT_OK);
 assert(!j[0].reference_valid&&!j[0].configured);
 assert(joint_controller_enable(&j[0])==JOINT_NOT_READY);
 read_status=PLATFORM_IO_ERROR;
 assert(joint_controller_update(&j[1])==JOINT_FAULT&&!j[1].enabled);
 assert(joint_controller_disable(&j[2])==JOINT_OK&&!j[2].enabled);
 c.parameters_verified=false;assert(joint_controller_configure(&j[0],&c)==JOINT_INVALID);
 c.parameters_verified=true;c.busy_timeout_ms=5;c.pid.dt=NAN;assert(joint_controller_configure(&j[0],&c)==JOINT_INVALID);
 c.pid.dt=0.001f;c.reference_raw=65534U;
 read_status=PLATFORM_OK;raw[1]=1U;
 assert(joint_controller_reset(&j[1])==JOINT_OK);
 assert(joint_controller_configure(&j[1],&c)==JOINT_OK);
 assert(joint_controller_update(&j[1])==JOINT_OK&&j[1].feedback.counts==1);
 assert(joint_controller_set_reference(&j[1],1)==JOINT_OK);
 assert(joint_controller_enable(&j[1])==JOINT_OK);
 assert(j[1].pid.integral==0&&j[1].diagnostics.reference==j[1].feedback.measured_q);
 write_status=PLATFORM_NOT_READY;
 assert(joint_controller_update(&j[1])==JOINT_FAULT&&j[1].pid.integral==0);
 assert(!j[1].enabled&&!j[1].diagnostics.write_accepted);
 assert(joint_controller_reset(&j[1])==JOINT_OK);
 assert(joint_controller_configure(&j[1],&c)==JOINT_OK);
 assert(joint_controller_update(&j[1])==JOINT_OK);
 assert(joint_controller_set_reference(&j[1],0)==JOINT_OK);
 assert(joint_controller_enable(&j[1])==JOINT_OK);write_status=PLATFORM_BUSY;
 for(i=0;i<4;i++)assert(joint_controller_update(&j[1])==JOINT_NOT_READY&&!j[1].fault);
 assert(joint_controller_update(&j[1])==JOINT_FAULT&&!j[1].enabled);
 puts("joint_controller: PASS");return 0;
}
