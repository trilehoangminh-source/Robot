#include <assert.h>
#include <stdio.h>
#include "state_machine.h"
int main(void){state_machine_t s;sm_guard_t g={true,true,true,true,0};
 state_machine_initialize(&s);assert(s.state==PROTO_INIT);
 assert(!state_machine_process(&s,SM_MOTION_START,&g));
 assert(state_machine_process(&s,SM_INIT_OK,&g));
 assert(state_machine_process(&s,SM_ENABLE,&g)&&s.actuator_enable);
 assert(!state_machine_process(&s,SM_MOTION_START,&g));
 assert(state_machine_process(&s,SM_HOME,&g)&&s.homing_permission);
 assert(state_machine_process(&s,SM_HOME_DONE,&g)&&s.homed);
 assert(state_machine_can_transition(&s,SM_MOTION_START,&g));
 assert(state_machine_process(&s,SM_MOTION_START,&g)&&state_machine_motion_allowed(&s));
 assert(state_machine_process(&s,SM_STOP,&g)&&!state_machine_motion_allowed(&s)&&!s.pending_motion);
 assert(!state_machine_process(&s,SM_MOTION_START,&g));
 assert(state_machine_process(&s,SM_ENABLE,&g)&&s.state==PROTO_READY);
 assert(state_machine_process(&s,SM_REFERENCE_LOST,&g)&&!s.homed);
 g.active_faults=FAULT_ENCODER;assert(state_machine_process(&s,SM_FAULT_DETECTED,&g));
 assert(!state_machine_process(&s,SM_RESET_FAULT,&g));g.active_faults=0;
 assert(state_machine_process(&s,SM_RESET_FAULT,&g)&&s.state==PROTO_IDLE&&!s.actuator_enable&&!s.homed);
 assert(!state_machine_process(&s,SM_MOTION_START,&g));
 assert(state_machine_process(&s,SM_ENABLE,&g));assert(state_machine_process(&s,SM_HOME,&g));
 assert(state_machine_process(&s,SM_HOME_FAIL,&g)&&s.state==PROTO_FAULT);
 puts("state_machine: PASS");return 0;}
