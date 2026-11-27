#include "robot.h"
#include <math.h>
static platform_status_t cached(void *ctx,uint8_t j,uint32_t step,uint32_t dt,platform_encoder_feedback_t *out){robot_t *r=ctx;(void)step;(void)dt;*out=r->feedback[j];return out->valid?PLATFORM_OK:PLATFORM_IO_ERROR;}
static platform_status_t write_motor(void *ctx,uint8_t j,float u){robot_t *r=ctx;return platform_set_motor_pwm(&r->platform,j,u);}
/* Controller cancellation routes through whole-robot policy on next manager check. */
static platform_status_t cancel(void *ctx,uint8_t j){robot_t *r=ctx;(void)j;r->stop_pending=true;return PLATFORM_OK;}
static const platform_ops_t controller_ops={0,0,0,write_motor,0,cached,cancel};
static bool all_homed(const robot_t *r){return r->homing.homed[0]&&r->homing.homed[1]&&r->homing.homed[2];}
static sm_guard_t guard(const robot_t *r){sm_guard_t g={0};g.safe=r->snapshot_valid&&!r->safety.latched_faults&&!r->execution_faults;g.homing_safe=g.safe&&r->machine.actuator_enable;g.reference_valid=all_homed(r);g.active_faults=(uint16_t)(r->safety.active_faults|r->execution_faults);g.recovery_allowed=g.safe;return g;}
void robot_request_stop(robot_t *r){sm_guard_t g;if(!r)return;trajectory_abort(&r->trajectory);r->command_pending=false;r->stop_pending=true;r->busy=false;r->reference_ack=r->sync_ack=false;homing_abort(&r->homing);gripper_abort(&r->gripper);g=guard(r);(void)state_machine_process(&r->machine,SM_STOP,&g);}
static bool stop_now(robot_t *r){unsigned i;platform_status_t s;
 for(i=0;i<3;i++){r->joint[i].enabled=false;r->joint[i].reference_valid=false;r->joint[i].busy_steps=0;(void)pid_reset(&r->joint[i].pid);}
 r->stop_result.output_inhibited=false;r->stop_result.physical_stop_confirmed=false;
 s=r->services.stop(r->services.context,&r->stop_result);
 r->stop_pending=s!=PLATFORM_OK;return s==PLATFORM_OK;
}
static void fault(robot_t *r,uint16_t bits){sm_guard_t g;r->execution_faults|=bits;robot_request_stop(r);g=guard(r);(void)state_machine_process(&r->machine,SM_FAULT_DETECTED,&g);(void)stop_now(r);}
bool robot_initialize(robot_t *r,const platform_t *p,const robot_services_t *svc){robot_t z={0};unsigned i;if(!r||!p||!p->ops||!p->ops->encoder_feedback||!svc||!svc->limits||!svc->enable||!svc->stop||!svc->reference)return false;
 *r=z;r->platform=*p;r->services=*svc;r->controller_platform.ops=&controller_ops;r->controller_platform.context=r;
 trajectory_initialize(&r->trajectory);state_machine_initialize(&r->machine);if(!gripper_initialize(&r->gripper,p))return false;
 for(i=0;i<3;i++)if(joint_controller_initialize(&r->joint[i],(uint8_t)i,&r->controller_platform)!=JOINT_OK)return false;r->initialized=true;return true;
}
bool robot_configure(robot_t *r,const robot_config_t *c){unsigned i;sm_guard_t g={true,false,false,false,0};
 if(!r||!r->initialized||!c||r->machine.state!=PROTO_INIT)return false;
 /* No real actuator operation until a deployment supplies a verified policy. */
 if(!c->simulation&&!c->stop_policy_verified)return false;
 if(!safety_initialize(&r->safety,&c->safety)||!homing_initialize(&r->homing,&c->homing))return false;
 for(i=0;i<3;i++){if(fabsf(c->joint[i].pid.dt-0.001f)>0.0000001f||joint_controller_configure(&r->joint[i],&c->joint[i])!=JOINT_OK)return false;}
 if(c->trajectory.verified&&!trajectory_configure(&r->trajectory,&c->trajectory))return false;
 r->config=*c;(void)gripper_configure(&r->gripper,c->gripper_pulses_verified);r->configured=true;return state_machine_process(&r->machine,SM_INIT_OK,&g);
}
bool robot_process_command(robot_t *r,const command_t *c,uint32_t now,frame_t *out){telemetry_snapshot_t t={0};uint8_t nack=0,id=COMM_MSG_ACK;sm_guard_t g;
 if(!r||!c||!out||!r->configured)return false;r->now_ms=now;t.request_id=c->id;g=guard(r);
 switch(c->id){
 case COMM_CMD_HEARTBEAT:r->heartbeat_valid=true;r->heartbeat_ms=now;return false;
 case COMM_CMD_PING:id=COMM_MSG_PONG;break;
 case COMM_CMD_GET_INFO:id=COMM_MSG_ROBOT_INFO;t.fw_major=1;break;
 case COMM_CMD_GET_STATE:if(!robot_get_telemetry(r,&t))nack=NACK_STATE;else id=COMM_MSG_STATE_REPLY;break;
 case COMM_CMD_STOP:robot_request_stop(r);break;
 case COMM_CMD_DISABLE:robot_request_stop(r);(void)state_machine_process(&r->machine,SM_DISABLE,&g);break;
 case COMM_CMD_SET_JOINT:nack=NACK_NOT_IMPLEMENTED;break;
 case COMM_CMD_TRAJECTORY_BEGIN:case COMM_CMD_TRAJECTORY_POINT:case COMM_CMD_TRAJECTORY_CLEAR:
 if(r->stop_pending||r->command_pending||!g.safe||r->machine.state!=PROTO_READY||!r->machine.homed)nack=NACK_STATE;
 else if(c->id==COMM_CMD_TRAJECTORY_BEGIN){if(!trajectory_begin(&r->trajectory,c->trajectory_id,c->point_count))nack=NACK_TRAJECTORY;}
 else if(c->id==COMM_CMD_TRAJECTORY_POINT){if(!trajectory_append(&r->trajectory,c->trajectory_id,c->index,c->time_ms,c->q))nack=NACK_TRAJECTORY;}
 else if(!trajectory_clear(&r->trajectory))nack=NACK_BUSY;
 break;
 case COMM_CMD_TRAJECTORY_START:{float q[3];unsigned i;for(i=0;i<3;i++)q[i]=r->feedback[i].q;
 if(!r->config.simulation)nack=NACK_NOT_IMPLEMENTED;
 else if(r->command_pending||r->stop_pending)nack=NACK_BUSY;
 else if(!state_machine_can_transition(&r->machine,SM_MOTION_START,&g))nack=NACK_STATE;
 else if(c->trajectory_id!=r->trajectory.id||!trajectory_validate(&r->trajectory,q))nack=NACK_TRAJECTORY;
 else{r->pending=*c;r->command_pending=true;}break;}

 case COMM_CMD_ENABLE:case COMM_CMD_HOME:case COMM_CMD_GRIPPER:case COMM_CMD_RESET_FAULT:
 if(r->stop_pending||r->command_pending)nack=NACK_BUSY;
 else if(c->id==COMM_CMD_ENABLE&&!state_machine_can_transition(&r->machine,SM_ENABLE,&g))nack=NACK_STATE;
 else if(c->id==COMM_CMD_HOME&&!state_machine_can_transition(&r->machine,SM_HOME,&g))nack=NACK_STATE;
 else if(c->id==COMM_CMD_GRIPPER&&(!g.safe||!r->machine.actuator_enable||c->gripper>1||!r->gripper.configured))nack=NACK_STATE;
 else if(c->id==COMM_CMD_RESET_FAULT&&(r->machine.state!=PROTO_FAULT||r->safety.active_faults||!r->stop_result.output_inhibited))nack=NACK_FAULT;
 else {r->pending=*c;r->command_pending=true;}break;
 default:nack=NACK_UNSUPPORTED;
 }
 if(nack){id=COMM_MSG_NACK;t.request_id=c->id;t.code=nack;}
 return telemetry_encode(id,c->seq,&t,out);
}
static void pending(robot_t *r){sm_guard_t g=guard(r);command_t c=r->pending;homing_input_t in={0};uint8_t j;
 if(!r->command_pending)return;r->command_pending=false;
 if(c.id==COMM_CMD_TRAJECTORY_START){float q[3];unsigned i;
 for(i=0;i<3;i++)q[i]=r->feedback[i].q;
 if(!state_machine_can_transition(&r->machine,SM_MOTION_START,&g)||!trajectory_start(&r->trajectory,c.trajectory_id,r->now_ms,q)){fault(r,FAULT_TRAJECTORY);return;}
 for(i=0;i<3;i++)if(joint_controller_update(&r->joint[i])!=JOINT_OK||joint_controller_set_reference(&r->joint[i],q[i])!=JOINT_OK||(!r->joint[i].enabled&&joint_controller_enable(&r->joint[i])!=JOINT_OK)){fault(r,FAULT_ENCODER);return;}
 (void)state_machine_process(&r->machine,SM_MOTION_START,&g);
 }
 else if(c.id==COMM_CMD_ENABLE){if(!state_machine_can_transition(&r->machine,SM_ENABLE,&g)||r->services.enable(r->services.context,true)!=PLATFORM_OK){fault(r,FAULT_MOTOR);return;}(void)state_machine_process(&r->machine,SM_ENABLE,&g);r->gripper.disable_requested=false;}
 else if(c.id==COMM_CMD_HOME){if(!state_machine_process(&r->machine,SM_HOME,&g))return;
 (void)homing_initialize(&r->homing,&r->config.homing);j=r->config.homing.order[0];in.permission=true;in.safe=g.safe;in.feedback_valid=r->feedback[j].valid;in.q=r->feedback[j].q;in.now_ms=r->now_ms;
 if(!homing_start(&r->homing,&in))fault(r,FAULT_HOMING);}
 else if(c.id==COMM_CMD_GRIPPER){if(gripper_request(&r->gripper,c.gripper!=0,g.safe&&r->machine.actuator_enable)!=PLATFORM_OK)fault(r,FAULT_MOTOR);}
 else if(c.id==COMM_CMD_RESET_FAULT){r->execution_faults=0;if(safety_reset_fault(&r->safety,true)){g=guard(r);(void)state_machine_process(&r->machine,SM_RESET_FAULT,&g);}}
}

bool robot_control_step(robot_t *r,uint32_t now,const safety_snapshot_t *external){
 safety_snapshot_t in;unsigned i;bool valid=true;sm_guard_t g;
 if(!r||!r->configured||!external)return false;r->now_ms=now;++r->step_id;in=*external;
 in.now_ms=now;in.heartbeat_valid=r->heartbeat_valid;in.heartbeat_ms=r->heartbeat_ms;
 in.operating=r->machine.actuator_enable;in.homing=r->machine.state==PROTO_HOMING;
 in.homing_authorized=r->machine.actuator_enable;in.homing_start_ms=r->homing.start_ms;in.reference_valid=all_homed(r);
 for(i=0;i<3;i++){
 platform_encoder_feedback_t f={0};robot_limit_t l={0};safety_joint_t *j=&in.joint[i];
 if(r->platform.ops->encoder_feedback(r->platform.context,(uint8_t)i,r->step_id,1,&f)!=PLATFORM_OK||!f.valid||!isfinite(f.q)||!isfinite(f.w)){valid=false;f.valid=false;}
 if(r->services.limits(r->services.context,(uint8_t)i,&l)!=PLATFORM_OK)l.valid=false;
 if(r->snapshot_valid&&r->feedback[i].reference_epoch!=f.reference_epoch&&r->machine.state!=PROTO_HOMING){robot_request_stop(r);r->machine.homed=false;}
 r->feedback[i]=f;r->limit[i]=l;j->feedback_valid=f.valid;j->measured_q=f.q;j->measured_w=f.w;j->raw_count=f.raw;
 j->estimated_q=r->joint[i].feedback.estimated_q;j->estimated_w=r->joint[i].feedback.estimated_w;j->tracking_error=r->joint[i].diagnostics.error;
 j->limit_valid=l.valid;j->raw_limit=l.raw_active;j->debounced_limit=l.debounced_active;j->limit_active=l.raw_active||l.debounced_active;
 j->homing_approach=in.homing&&r->homing.config.order[r->homing.index]==i&&(r->homing.phase==HOME_SEEK||r->homing.phase==HOME_SLOW||r->homing.phase==HOME_CHECK||r->homing.phase==HOME_REFERENCE);
 if(in.homing&&r->homing.config.order[r->homing.index]==i&&(r->homing.phase==HOME_RELEASE||r->homing.phase==HOME_BACKOFF)){j->limit_direction_known=true;j->moving_into_limit=false;}
 }
 r->snapshot_valid=valid;
 if(!safety_update(&r->safety,&in)||r->safety.latched_faults){fault(r,r->safety.latched_faults?r->safety.latched_faults:FAULT_ENCODER);return false;}
 if(r->stop_pending){if(!stop_now(r)){fault(r,FAULT_MOTOR);return false;}return true;}
 pending(r);if(r->machine.state==PROTO_FAULT)return false;
 if(r->machine.state==PROTO_HOMING){
 homing_input_t h={0};uint8_t j=r->homing.config.order[r->homing.index];platform_status_t status;
 h.now_ms=now;h.permission=r->safety.homing_permission;h.safe=true;h.feedback_valid=r->feedback[j].valid;h.q=r->feedback[j].q;
 h.raw_active=r->limit[j].raw_active;h.debounced_active=r->limit[j].debounced_active;h.switch_valid=r->limit[j].valid;
 h.stop_confirmed=external->physical_stop_valid&&external->physical_stop;
 h.reference_committed=r->reference_ack;h.controller_synchronized=r->sync_ack;
 if(!homing_update(&r->homing,&h)){fault(r,FAULT_HOMING);return false;}
 if(r->homing.intent.reference_request&&!r->reference_ack&&h.stop_confirmed){
 uint32_t epoch=r->feedback[j].reference_epoch;
 if(!h.stop_confirmed||r->services.reference(r->services.context,j,r->homing.intent.home_angle)!=PLATFORM_OK){fault(r,FAULT_ENCODER);return false;}
 /* Refresh after commit using next sampling epoch, then synchronize Controller. */
 if(r->platform.ops->encoder_feedback(r->platform.context,j,r->step_id,1,&r->feedback[j])!=PLATFORM_OK||!r->feedback[j].valid||r->feedback[j].reference_epoch==epoch){fault(r,FAULT_ENCODER);return false;}
 (void)joint_controller_reset(&r->joint[j]);
 if(joint_controller_configure(&r->joint[j],&r->config.joint[j])!=JOINT_OK||joint_controller_update(&r->joint[j])!=JOINT_OK){fault(r,FAULT_ENCODER);return false;}
 r->reference_ack=r->sync_ack=true;
 }else if(r->homing.phase!=HOME_REFERENCE)r->reference_ack=r->sync_ack=false;
 if(r->homing.intent.motion){if(r->services.enable(r->services.context,true)!=PLATFORM_OK){fault(r,FAULT_MOTOR);return false;}status=platform_set_motor_pwm(&r->platform,j,r->homing.intent.output);
 if(status==PLATFORM_BUSY){if(!r->busy){r->busy=true;r->busy_since=now;}if((uint32_t)(now-r->busy_since)>=r->config.joint[j].busy_timeout_ms){fault(r,FAULT_MOTOR);return false;}}
 else if(status!=PLATFORM_OK){fault(r,FAULT_MOTOR);return false;}else r->busy=false;
 }else {robot_stop_result_t result={0};if(r->services.stop(r->services.context,&result)!=PLATFORM_OK){fault(r,FAULT_MOTOR);return false;}r->stop_result=result;}

 if(r->homing.phase==HOME_DONE){g=guard(r);if(!state_machine_process(&r->machine,SM_HOME_DONE,&g)){fault(r,FAULT_HOMING);return false;}r->event=EVENT_HOMING_DONE;}
 }else{
 float q[3],w[3],reference[3];bool executing=r->trajectory.state==TRAJ_RUNNING||r->trajectory.state==TRAJ_SETTLING||r->trajectory.state==TRAJ_DONE;
 for(i=0;i<3;i++){q[i]=r->feedback[i].q;w[i]=r->feedback[i].w;}
 if(executing&&(!r->machine.homed||!r->machine.actuator_enable||!r->safety.motion_permission||!trajectory_update(&r->trajectory,now,q,w,reference))){fault(r,FAULT_TRAJECTORY);return false;}
 for(i=0;i<3;i++){
 if(executing&&joint_controller_set_reference(&r->joint[i],reference[i])!=JOINT_OK){fault(r,FAULT_TRAJECTORY);return false;}
 if(!executing&&r->joint[i].enabled&&r->machine.state!=PROTO_READY){fault(r,FAULT_TRAJECTORY);return false;}
 if(all_homed(r)&&joint_controller_update(&r->joint[i])==JOINT_FAULT){fault(r,FAULT_ENCODER);return false;}
 }
 if(r->trajectory.state==TRAJ_DONE&&r->machine.state==PROTO_RUNNING){g=guard(r);if(!state_machine_process(&r->machine,SM_MOTION_DONE,&g)){fault(r,FAULT_TRAJECTORY);return false;}r->event=EVENT_TRAJECTORY_DONE;}
 }return true;
}
void robot_application_step(robot_t *r,uint32_t now){if(r)r->now_ms=now;}
uint8_t robot_get_state(const robot_t *r){return r?r->machine.state:PROTO_FAULT;}
bool robot_get_telemetry(const robot_t *r,telemetry_snapshot_t *out){telemetry_snapshot_t t={0};unsigned i;
 if(!r||!out||!r->snapshot_valid)return false;
 t.feedback_valid=true;t.time_ms=r->now_ms;t.state=r->machine.state;t.fault_flags=(uint16_t)(r->safety.latched_faults|r->execution_faults);t.gripper=r->gripper.command;
 for(i=0;i<3;i++){if(!r->feedback[i].valid)return false;t.q[i]=r->feedback[i].q;t.w[i]=r->feedback[i].w;
 t.reference[i]=r->joint[i].diagnostics.reference;t.error[i]=r->joint[i].diagnostics.error;t.output[i]=r->joint[i].diagnostics.requested_output;
 if(r->limit[i].debounced_active)t.limit_flags|=(uint8_t)(1U<<i);if(r->homing.homed[i])t.homing_flags|=(uint8_t)(1U<<i);}
 *out=t;return true;
}
bool robot_get_pending_event(robot_t *r,uint8_t *event){if(!r||!event||!r->event)return false;*event=r->event;r->event=0;return true;}
