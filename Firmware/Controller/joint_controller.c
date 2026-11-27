#include "joint_controller.h"
#include <math.h>
#include <limits.h>
static joint_status_t fail(joint_controller_t *j) {
 if(j->platform.ops->cancel_motor)(void)j->platform.ops->cancel_motor(j->platform.context,j->joint);
 j->busy_steps=0;j->fault=true;j->enabled=false;j->reference_valid=false;j->feedback.valid=false;
 j->diagnostics.write_accepted=false;return JOINT_FAULT;
}
joint_status_t joint_controller_initialize(joint_controller_t *j,uint8_t id,const platform_t *p){
 joint_controller_t empty={0};
 if(!j||!p||!p->ops||id>=ROBOT_JOINT_COUNT)return JOINT_INVALID;
 *j=empty;j->joint=id;j->platform=*p;j->initialized=true;return JOINT_OK;
}
joint_status_t joint_controller_configure(joint_controller_t *j,const joint_config_t *c){
 robot_pid_t pid={0};kalman_t k={0};const robot_joint_calibration_t *r;
 if(!j||!j->initialized||!c)return JOINT_INVALID;
 if(j->enabled||j->fault)return JOINT_NOT_READY;
 r=&c->calibration;
 if(!c->busy_timeout_ms||!c->parameters_verified||!r->calibration_valid||!r->limits_valid||!r->encoder_cpr||
 (r->encoder_sign!=1&&r->encoder_sign!=-1)||!isfinite(r->home_angle_rad)||
 !isfinite(r->min_angle_rad)||!isfinite(r->max_angle_rad)||r->min_angle_rad>=r->max_angle_rad||
 c->pid.out_min< -1||c->pid.out_max>1||
 (j->joint!=0&&c->reference_raw>65535U)||
 !pid_initialize(&pid,&c->pid)||!kalman_initialize(&k,&c->kalman))return JOINT_INVALID;
 j->config=*c;j->pid=pid;j->kalman=k;j->configured=true;j->sampled=false;
 j->reference_valid=false;j->feedback.valid=false;j->feedback.raw=c->reference_raw;
 j->feedback.counts=r->home_count;return JOINT_OK;
}
joint_status_t joint_controller_reset(joint_controller_t *j){
 if(!j||!j->initialized)return JOINT_INVALID;
 if(j->platform.ops->cancel_motor)(void)j->platform.ops->cancel_motor(j->platform.context,j->joint);
 j->busy_steps=0;j->enabled=false;j->fault=false;j->configured=false;j->reference_valid=false;
 j->sampled=false;j->feedback.valid=false;j->diagnostics.write_accepted=false;
 return JOINT_OK;
}
joint_status_t joint_controller_enable(joint_controller_t *j){
 if(!j||!j->initialized)return JOINT_INVALID;
 if(j->fault)return JOINT_FAULT;
 if(!j->configured||!j->reference_valid||!j->feedback.valid)return JOINT_NOT_READY;
 if(j->enabled)return JOINT_NOT_READY;
 /* Start in hold at current measured position; caller supplies motion reference afterwards. */
 j->diagnostics.reference=j->feedback.measured_q;
 if(!pid_reset(&j->pid)||!kalman_reset(&j->kalman,j->feedback.measured_q))return fail(j);
 j->enabled=true;return JOINT_OK;
}
joint_status_t joint_controller_disable(joint_controller_t *j){
 if(!j||!j->initialized)return JOINT_INVALID;
 if(j->platform.ops->cancel_motor)(void)j->platform.ops->cancel_motor(j->platform.context,j->joint);
 j->busy_steps=0;j->enabled=false;j->reference_valid=false;j->diagnostics.write_accepted=false;
 (void)pid_reset(&j->pid);return JOINT_OK;
}
joint_status_t joint_controller_set_reference(joint_controller_t *j,float q){
 if(!j||!j->initialized||!isfinite(q))return JOINT_INVALID;
 if(j->fault)return JOINT_FAULT;
 if(!j->configured)return JOINT_NOT_READY;
 if(q<j->config.calibration.min_angle_rad||q>j->config.calibration.max_angle_rad)return JOINT_INVALID;
 j->diagnostics.reference=q;j->reference_valid=true;return JOINT_OK;
}
joint_status_t joint_controller_update(joint_controller_t *j){
 platform_encoder_feedback_t f;float q,w,u;uint32_t dt_ms;
 robot_pid_t pid;kalman_t k;platform_status_t s;
 if(!j||!j->initialized)return JOINT_INVALID;
 if(j->fault)return JOINT_FAULT;
 if(!j->configured)return JOINT_NOT_READY;
 j->diagnostics.write_accepted=false;j->diagnostics.applied_output_known=false;
 if(!j->platform.ops->encoder_feedback)return fail(j);
 dt_ms=(uint32_t)(j->config.pid.dt*1000.0f+0.5f);
 if(!dt_ms)return fail(j);
 s=j->platform.ops->encoder_feedback(j->platform.context,j->joint,++j->step_id,dt_ms,&f);
 j->diagnostics.platform_status=s;
 if(s!=PLATFORM_OK||!f.valid||!isfinite(f.q)||!isfinite(f.w))return fail(j);
 if(j->sampled&&j->epoch!=f.reference_epoch){
 (void)joint_controller_disable(j);(void)pid_reset(&j->pid);
 if(!kalman_reset(&j->kalman,f.q))return fail(j);
 }
 j->epoch=f.reference_epoch;q=f.q;w=f.w;
 if(q<j->config.calibration.min_angle_rad||q>j->config.calibration.max_angle_rad)return fail(j);
 k=j->kalman;
 if(!kalman_update(&k,j->config.pid.dt,q))return fail(j);
 j->feedback.raw=f.raw;j->feedback.counts=f.count;j->feedback.measured_q=q;j->feedback.measured_w=w;
 j->feedback.estimated_q=k.q;j->feedback.estimated_w=k.w;j->feedback.valid=true;j->sampled=true;j->kalman=k;
 if(!j->enabled)return JOINT_OK;
 if(!j->reference_valid||!isfinite(j->diagnostics.reference))return fail(j);
 pid=j->pid;
 if(!pid_update(&pid,j->diagnostics.reference,k.q,k.w,&u))return fail(j);
 j->diagnostics.error=j->diagnostics.reference-k.q;j->diagnostics.requested_output=u;
 s=platform_set_motor_pwm(&j->platform,j->joint,u);j->diagnostics.platform_status=s;
 if(s==PLATFORM_BUSY){
 ++j->busy_steps;
 if((float)j->busy_steps*j->config.pid.dt*1000.0f>=(float)j->config.busy_timeout_ms)return fail(j);
 return JOINT_NOT_READY;
 }
 if(s!=PLATFORM_OK)return fail(j);
 j->busy_steps=0;
 j->pid=pid;j->diagnostics.write_accepted=true;return JOINT_OK;
}
joint_status_t joint_controller_get_status(const joint_controller_t *j){
 if(!j||!j->initialized)return JOINT_INVALID;
 return j->fault?JOINT_FAULT:(j->configured?JOINT_OK:JOINT_NOT_READY);
}
joint_status_t joint_controller_get_feedback(const joint_controller_t *j,joint_feedback_t *out){
 if(!j||!out)return JOINT_INVALID;*out=j->feedback;return joint_controller_get_status(j);
}
joint_status_t joint_controller_get_diagnostics(const joint_controller_t *j,joint_diagnostics_t *out){
 if(!j||!out)return JOINT_INVALID;*out=j->diagnostics;return joint_controller_get_status(j);
}
