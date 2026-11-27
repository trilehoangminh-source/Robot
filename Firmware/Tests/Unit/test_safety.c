#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "safety.h"
int main(void){safety_t s;safety_config_t c={0};safety_snapshot_t in={0};unsigned i;
 assert(!safety_initialize(&s,&c));c.verified=true;c.tracking_ms=10;c.homing_ms=100;
 for(i=0;i<3;i++){c.min_q[i]=-2;c.max_q[i]=2;c.tracking_threshold[i]=0.5f;in.joint[i].feedback_valid=true;in.joint[i].limit_valid=true;}
 assert(safety_initialize(&s,&c));in.reference_valid=true;
 assert(safety_update(&s,&in)&&s.motion_permission&&!s.physical_stop_confirmed);
 in.operating=true;assert(safety_update(&s,&in)&&(s.active_faults&FAULT_COMM));
 in.heartbeat_valid=true;in.heartbeat_ms=UINT32_MAX-10;in.now_ms=5;
 assert(safety_update(&s,&in)&&s.active_faults==0&&s.latched_faults==FAULT_COMM);
 assert(safety_reset_fault(&s,true)&&!s.motion_permission);
 in.joint[0].measured_q=3;in.joint[1].feedback_valid=false;in.deadline_missed=true;
 in.trajectory_fault=true;in.estop_valid=true;in.estop_active=true;
 in.joint[2].driver_fault_valid=true;in.joint[2].driver_fault=true;
 assert(safety_update(&s,&in));assert((s.active_faults&(FAULT_POSITION|FAULT_ENCODER|FAULT_DEADLINE|FAULT_TRAJECTORY|FAULT_ESTOP|FAULT_MOTOR))==(FAULT_POSITION|FAULT_ENCODER|FAULT_DEADLINE|FAULT_TRAJECTORY|FAULT_ESTOP|FAULT_MOTOR));
 assert(!safety_reset_fault(&s,true));
 in.joint[0].measured_q=NAN;assert(safety_update(&s,&in)&&(s.active_faults&FAULT_ENCODER));
 in.joint[0].measured_q=0;in.joint[1].feedback_valid=true;in.deadline_missed=false;in.trajectory_fault=false;in.estop_active=false;in.joint[2].driver_fault=false;
 in.joint[0].tracking_error=1;in.now_ms=6;assert(safety_update(&s,&in)&&!(s.active_faults&FAULT_TRACKING));
 in.now_ms=16;assert(safety_update(&s,&in)&&(s.active_faults&FAULT_TRACKING));
 in.operating=false;in.homing=true;in.homing_authorized=true;in.homing_start_ms=16;
 in.joint[0].limit_active=true;in.joint[0].homing_approach=true;
 assert(safety_update(&s,&in)&&!(s.active_faults&FAULT_SWITCH));
 in.homing=false;assert(safety_update(&s,&in)&&(s.active_faults&FAULT_SWITCH));
 in.joint[0].limit_direction_known=true;in.joint[0].moving_into_limit=false;
 assert(safety_update(&s,&in)&&!(s.active_faults&FAULT_SWITCH));
 in.homing=true;in.now_ms=116;assert(safety_update(&s,&in)&&(s.active_faults&FAULT_HOMING));
 in.now_ms=600;assert(safety_update(&s,&in)&&(s.active_faults&FAULT_COMM));
 puts("safety: PASS");return 0;}
