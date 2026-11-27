#include "safety.h"
#include <math.h>
static bool valid(const safety_config_t *c){unsigned i;if(!c||!c->verified||!c->tracking_ms||!c->homing_ms||c->tracking_ms>0x7fffffffU||c->homing_ms>0x7fffffffU)return false;
 for(i=0;i<3;i++)if(!isfinite(c->min_q[i])||!isfinite(c->max_q[i])||c->min_q[i]>=c->max_q[i]||!isfinite(c->tracking_threshold[i])||c->tracking_threshold[i]<=0)return false;return true;}
bool safety_initialize(safety_t *s,const safety_config_t *c){safety_t z={0};if(!s)return false;*s=z;if(!valid(c))return false;s->config=*c;s->configured=true;return true;}
bool safety_update(safety_t *s,const safety_snapshot_t *in){unsigned i;uint16_t f=0;bool ready=true;
 if(!s||!in)return false;
 s->motion_permission=false;s->homing_permission=false;s->stop_requested=true;
 s->physical_stop_confirmed=in->physical_stop_valid&&in->physical_stop;
 s->output_inhibited=in->output_inhibited_valid&&in->output_inhibited;
 s->evaluated=false;
 if(!s->configured||!valid(&s->config))return false;
 if(in->operating||in->homing){if(!in->heartbeat_valid||(uint32_t)(in->now_ms-in->heartbeat_ms)>=COMM_TIMEOUT_MS)f|=FAULT_COMM;}
 if(in->homing&&(uint32_t)(in->now_ms-in->homing_start_ms)>=s->config.homing_ms)f|=FAULT_HOMING;
 if(in->deadline_missed)f|=FAULT_DEADLINE;if(in->trajectory_fault)f|=FAULT_TRAJECTORY;
 if(in->estop_valid&&in->estop_active)f|=FAULT_ESTOP;
 for(i=0;i<3;i++){
 const safety_joint_t *j=&in->joint[i];bool finite=isfinite(j->measured_q)&&isfinite(j->measured_w)&&isfinite(j->estimated_q)&&isfinite(j->estimated_w)&&isfinite(j->tracking_error);
 if(!j->feedback_valid||!finite){f|=FAULT_ENCODER;ready=false;}
 if(j->feedback_valid&&isfinite(j->measured_q)&&in->reference_valid&&(j->measured_q<s->config.min_q[i]||j->measured_q>s->config.max_q[i]))f|=FAULT_POSITION;
 if(!j->limit_valid){ready=false;f|=FAULT_SWITCH;}
 if(j->limit_valid&&j->limit_active){
 bool allowed=in->homing&&in->homing_authorized&&j->homing_approach;
 if(!allowed&&(!j->limit_direction_known||j->moving_into_limit))f|=FAULT_SWITCH;
 }
 if(j->controller_fault)f|=FAULT_ENCODER;
 if(j->driver_fault_valid&&j->driver_fault)f|=FAULT_MOTOR;
 if(in->operating&&finite&&fabsf(j->tracking_error)>s->config.tracking_threshold[i]){
 if(!s->tracking[i]){s->tracking[i]=true;s->tracking_start[i]=in->now_ms;}
 if((uint32_t)(in->now_ms-s->tracking_start[i])>=s->config.tracking_ms)f|=FAULT_TRACKING;
 }else s->tracking[i]=false;
 }
 s->active_faults=f;s->latched_faults|=f;s->evaluated=true;
 s->homing_permission=ready&&!s->latched_faults&&in->homing_authorized;
 s->motion_permission=ready&&!s->latched_faults&&in->reference_valid;
 s->stop_requested=!ready||s->latched_faults!=0;return true;
}
bool safety_reset_fault(safety_t *s,bool recovery){if(!s||!s->configured||!s->evaluated||!valid(&s->config)||!recovery||s->active_faults)return false;s->latched_faults=0;s->motion_permission=false;s->homing_permission=false;s->stop_requested=true;return true;}
