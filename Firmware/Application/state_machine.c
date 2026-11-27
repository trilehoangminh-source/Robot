#include "state_machine.h"
void state_machine_initialize(state_machine_t *s){state_machine_t z={0};if(s)*s=z;}
static void revoke(state_machine_t *s){s->motion_permission=false;s->homing_permission=false;s->pending_motion=false;s->stop_requested=true;}
bool state_machine_process(state_machine_t *s,sm_event_t e,const sm_guard_t *g){
 if(!s||!g||s->state>PROTO_FAULT)return false;
 if(s->homed&&!g->reference_valid){s->homed=false;revoke(s);if(s->state!=PROTO_FAULT)s->state=PROTO_STOPPED;}
 if(e==SM_FAULT_DETECTED||e==SM_INIT_FAIL||e==SM_HOME_FAIL){
 s->latched_faults|=g->active_faults;s->state=PROTO_FAULT;s->actuator_enable=false;s->homed=false;revoke(s);return true;}
 if(e==SM_REFERENCE_LOST){s->homed=false;revoke(s);if(s->state!=PROTO_FAULT)s->state=PROTO_STOPPED;return true;}
 if(e==SM_DISABLE){s->actuator_enable=false;revoke(s);if(s->state!=PROTO_FAULT&&s->state!=PROTO_INIT)s->state=PROTO_STOPPED;return true;}
 if(e==SM_STOP){if(s->state==PROTO_INIT)return false;revoke(s);if(s->state!=PROTO_FAULT)s->state=PROTO_STOPPED;return true;}
 if(e==SM_RESET_FAULT){if(s->state!=PROTO_FAULT||g->active_faults||!g->recovery_allowed||!g->safe)return false;
 s->latched_faults=0;s->homed=false;s->actuator_enable=false;s->state=PROTO_IDLE;revoke(s);return true;}
 if(s->state==PROTO_FAULT||g->active_faults)return false;
 switch(e){
 case SM_INIT_OK:if(s->state!=PROTO_INIT||!g->safe)return false;s->state=PROTO_IDLE;break;
 case SM_ENABLE:if((s->state!=PROTO_IDLE&&s->state!=PROTO_STOPPED&&s->state!=PROTO_READY)||!g->safe)return false;
 s->actuator_enable=true;s->stop_requested=false;s->state=s->homed&&g->reference_valid?PROTO_READY:PROTO_IDLE;break;
 case SM_HOME:if(!s->actuator_enable||!g->safe||!g->homing_safe||(s->state!=PROTO_IDLE&&s->state!=PROTO_READY&&s->state!=PROTO_STOPPED))return false;
 s->homed=false;s->motion_permission=false;s->homing_permission=true;s->state=PROTO_HOMING;s->stop_requested=false;break;
 case SM_HOME_DONE:if(s->state!=PROTO_HOMING||!g->safe||!g->reference_valid)return false;
 s->homed=true;s->homing_permission=false;s->state=PROTO_READY;break;
 case SM_MOTION_START:if(s->state!=PROTO_READY||!s->homed||!s->actuator_enable||!g->safe||!g->reference_valid)return false;
 s->motion_permission=true;s->pending_motion=true;s->state=PROTO_RUNNING;break;
 case SM_MOTION_DONE:if(s->state!=PROTO_RUNNING)return false;s->motion_permission=false;s->pending_motion=false;s->state=PROTO_READY;break;
 default:return false;
 }return true;
}
bool state_machine_can_transition(const state_machine_t *s,sm_event_t e,const sm_guard_t *g){state_machine_t copy;if(!s)return false;copy=*s;return state_machine_process(&copy,e,g);}
uint8_t state_machine_get_state(const state_machine_t *s){return s?s->state:PROTO_FAULT;}
bool state_machine_motion_allowed(const state_machine_t *s){return s&&s->state==PROTO_RUNNING&&s->homed&&s->actuator_enable&&s->motion_permission&&!s->latched_faults;}
