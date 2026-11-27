#include <assert.h>
#include <stdio.h>
#include "homing.h"
int main(void){homing_t h;homing_config_t c={0};homing_input_t in={0};unsigned j;
 c.verified=true;c.total_timeout_ms=10000;for(j=0;j<3;j++){c.order[j]=(uint8_t)j;c.joint[j].direction=1;c.joint[j].seek=0.2f;c.joint[j].slow=0.1f;c.joint[j].backoff=0.2f;c.joint[j].backoff_rad=0.1f;c.joint[j].max_travel=2;c.joint[j].home_angle=0.4f;c.joint[j].phase_timeout_ms=1000;}
 assert(homing_initialize(&h,&c));assert(!homing_start(&h,&in));in.permission=in.safe=in.feedback_valid=in.switch_valid=true;
 assert(homing_start(&h,&in));
 for(j=0;j<3;j++){
 in.raw_active=in.debounced_active=false;in.q=0;in.now_ms++;assert(homing_update(&h,&in));
 in.now_ms++;assert(homing_update(&h,&in)&&h.intent.motion);
 in.raw_active=true;in.now_ms++;assert(homing_update(&h,&in)&&!h.intent.motion);
 in.debounced_active=in.stop_confirmed=true;in.now_ms++;assert(homing_update(&h,&in)&&h.phase==HOME_BACKOFF);
 in.raw_active=in.debounced_active=false;in.q=-0.2f;in.now_ms++;assert(homing_update(&h,&in)&&h.phase==HOME_SLOW);
 in.raw_active=in.debounced_active=true;in.now_ms++;assert(homing_update(&h,&in)&&h.phase==HOME_REFERENCE);
 in.now_ms++;assert(homing_update(&h,&in)&&h.intent.reference_request&&!h.homed[j]);
 in.reference_committed=in.controller_synchronized=true;in.now_ms++;assert(homing_update(&h,&in)&&h.homed[j]);in.reference_committed=in.controller_synchronized=false;
 }
 assert(h.phase==HOME_DONE);homing_abort(&h);assert(h.phase==HOME_ABORTED&&!h.homed[0]);
 assert(homing_initialize(&h,&c));in.now_ms=UINT32_MAX-5;assert(homing_start(&h,&in));in.now_ms=1000;assert(!homing_update(&h,&in)&&h.phase==HOME_FAULT);
 puts("homing: PASS");return 0;}
