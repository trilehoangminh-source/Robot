#include <assert.h>
#include <stdio.h>
#include "robot.h"
#include "communication_task.h"
static unsigned sent;
static bsp_status_t tx(const uint8_t *data,uint16_t size){frame_t f;assert(frame_decode(data,size,&f)==FRAME_OK);assert(f.seq==44||f.seq==45);sent++;return BSP_OK;}
static bsp_status_t io_ok(void){return BSP_OK;}
static platform_sim_state_t sim;static robot_limit_t limits[3];static unsigned starts,stops,refs;
static platform_status_t get_limit(void *c,uint8_t j,robot_limit_t *l){(void)c;*l=limits[j];return PLATFORM_OK;}
static platform_status_t enable(void *c,bool on){(void)c;(void)on;starts++;return PLATFORM_OK;}
static platform_status_t stop(void *c,robot_stop_result_t *out){(void)c;stops++;out->output_inhibited=true;out->physical_stop_confirmed=false;return PLATFORM_OK;}
static platform_status_t reference(void *c,uint8_t j,float q){(void)c;refs++;sim.feedback[j].q=q;sim.feedback[j].reference_epoch++;return PLATFORM_OK;}
int main(void){robot_t r;robot_config_t c={0};platform_t p;robot_services_t svc={get_limit,enable,stop,reference,0};safety_snapshot_t ext={0};command_t cmd={0};frame_t reply;unsigned i,j;uint32_t now=0;
 assert(platform_sim_bind(&p,&sim)==PLATFORM_OK);sim.ready=true;
 c.trajectory.verified=true;c.trajectory.settle_ms=2;
 for(i=0;i<3;i++){c.trajectory.min_q[i]=-10;c.trajectory.max_q[i]=10;c.trajectory.max_velocity[i]=1;c.trajectory.max_acceleration[i]=1000;c.trajectory.start_tolerance[i]=0.001f;c.trajectory.complete_error[i]=0.001f;c.trajectory.complete_velocity[i]=0.01f;}
 c.simulation=true;c.gripper_pulses_verified=true;c.safety.verified=true;c.safety.tracking_ms=10;c.safety.homing_ms=1000;c.homing.verified=true;c.homing.total_timeout_ms=1000;
 for(i=0;i<3;i++){sim.feedback[i].valid=true;limits[i].valid=true;c.safety.min_q[i]=-10;c.safety.max_q[i]=10;c.safety.tracking_threshold[i]=1;
 c.joint[i].parameters_verified=true;c.joint[i].busy_timeout_ms=10;c.joint[i].calibration.calibration_valid=1;c.joint[i].calibration.limits_valid=1;c.joint[i].calibration.encoder_cpr=1000;c.joint[i].calibration.encoder_sign=1;c.joint[i].calibration.min_angle_rad=-10;c.joint[i].calibration.max_angle_rad=10;
 c.joint[i].pid.dt=0.001f;c.joint[i].pid.kp=1;c.joint[i].pid.out_min=-1;c.joint[i].pid.out_max=1;c.joint[i].pid.i_min=-1;c.joint[i].pid.i_max=1;c.joint[i].kalman.r=0.01f;c.joint[i].kalman.p00=1;c.joint[i].kalman.p11=1;
 c.homing.order[i]=(uint8_t)i;c.homing.joint[i].direction=1;c.homing.joint[i].seek=0.2f;c.homing.joint[i].slow=0.1f;c.homing.joint[i].backoff=0.1f;c.homing.joint[i].backoff_rad=0.1f;c.homing.joint[i].max_travel=2;c.homing.joint[i].home_angle=0.3f;c.homing.joint[i].phase_timeout_ms=100;}
 assert(robot_initialize(&r,&p,&svc)&&robot_configure(&r,&c));assert(starts==0&&stops==0&&robot_get_state(&r)==PROTO_IDLE);
 {
 communication_task_t comm;communication_io_t io={tx,io_ok,io_ok};frame_t request={0},response;uint8_t bytes[136];size_t length;command_t queued;
 assert(robot_control_step(&r,now++,&ext));shared_data_initialize();{shared_snapshot_t ready={0};shared_snapshot_publish(&ready);}assert(communication_initialize(&comm,&io));request.id=COMM_CMD_ENABLE;request.seq=44;
 assert(frame_encode(&request,bytes,sizeof bytes,&length)==FRAME_OK);assert(communication_feed(&comm,bytes,length,0));assert(comm.pending&&sent==0);
 assert(communication_feed(&comm,bytes,length,0));assert(shared_command_pop(&queued));assert(!shared_command_pop(&cmd));assert(starts==0);
 assert(robot_process_command(&r,&queued,0,&response));assert(response.id==COMM_MSG_ACK&&response.seq==44&&response.payload[1]==0);assert(shared_response_push(&response));
 communication_poll(&comm,0,false,false,false);assert(sent==1&&starts==0&&r.command_pending);
 request.id=COMM_CMD_STOP;request.seq=45;assert(frame_encode(&request,bytes,sizeof bytes,&length)==FRAME_OK);assert(communication_feed(&comm,bytes,length,1));assert(shared_stop_command_take(&queued));assert(robot_process_command(&r,&queued,1,&response));shared_stop_response(&response);
 assert(!r.command_pending&&!r.machine.motion_permission&&starts==0);communication_poll(&comm,1,false,false,false);assert(sent==2);
 /* Ordinary retry after STOP only replays its response, never re-enables. */
 request.id=COMM_CMD_ENABLE;request.seq=44;assert(frame_encode(&request,bytes,sizeof bytes,&length)==FRAME_OK);assert(communication_feed(&comm,bytes,length,2));assert(!shared_command_pop(&queued)&&!r.command_pending&&starts==0);
 assert(robot_initialize(&r,&p,&svc)&&robot_configure(&r,&c));
 }
 assert(robot_control_step(&r,now++,&ext));cmd.id=COMM_CMD_HEARTBEAT;assert(!robot_process_command(&r,&cmd,now,&reply));
 cmd.id=COMM_CMD_SET_JOINT;assert(robot_process_command(&r,&cmd,now,&reply)&&reply.id==COMM_MSG_NACK);
 cmd.id=COMM_CMD_ENABLE;assert(robot_process_command(&r,&cmd,now,&reply)&&reply.id==COMM_MSG_ACK);assert(starts==0);assert(robot_control_step(&r,now++,&ext)&&starts==1);
 cmd.id=COMM_CMD_HOME;assert(robot_process_command(&r,&cmd,now,&reply)&&reply.id==COMM_MSG_ACK);assert(robot_control_step(&r,now++,&ext));
 ext.physical_stop_valid=ext.physical_stop=true;
 for(j=0;j<3;j++){
 if(j)assert(robot_control_step(&r,now++,&ext));
 assert(robot_control_step(&r,now++,&ext));
 limits[j].raw_active=limits[j].debounced_active=true;assert(robot_control_step(&r,now++,&ext));
 limits[j].raw_active=limits[j].debounced_active=false;sim.feedback[j].q=-0.2f;assert(robot_control_step(&r,now++,&ext));
 limits[j].raw_active=limits[j].debounced_active=true;assert(robot_control_step(&r,now++,&ext));assert(robot_control_step(&r,now++,&ext));
 assert(robot_control_step(&r,now++,&ext));limits[j].raw_active=limits[j].debounced_active=false;
 }
 assert(robot_control_step(&r,now++,&ext));assert(refs==3&&r.machine.homed&&robot_get_state(&r)==PROTO_READY);
 {uint8_t event;assert(robot_get_pending_event(&r,&event)&&event==EVENT_HOMING_DONE);assert(!robot_get_pending_event(&r,&event));}
 cmd.id=COMM_CMD_TRAJECTORY_BEGIN;cmd.trajectory_id=3;cmd.point_count=2;
 assert(robot_process_command(&r,&cmd,now,&reply)&&reply.id==COMM_MSG_ACK);
 cmd.id=COMM_CMD_TRAJECTORY_POINT;cmd.index=0;cmd.time_ms=0;for(i=0;i<3;i++)cmd.q[i]=0.3f;
 assert(robot_process_command(&r,&cmd,now,&reply)&&reply.id==COMM_MSG_ACK);
 cmd.index=1;cmd.time_ms=10;cmd.q[0]=0.31f;
 assert(robot_process_command(&r,&cmd,now,&reply)&&reply.id==COMM_MSG_ACK);
 cmd.id=COMM_CMD_TRAJECTORY_START;assert(robot_process_command(&r,&cmd,now,&reply)&&reply.id==COMM_MSG_ACK);
 assert(robot_control_step(&r,now++,&ext)&&r.machine.state==PROTO_RUNNING);
 assert(robot_control_step(&r,now+11,&ext)&&r.trajectory.state==TRAJ_SETTLING);
 assert(r.machine.state==PROTO_RUNNING);sim.feedback[0].q=0.31f;
 assert(robot_control_step(&r,now+12,&ext));assert(robot_control_step(&r,now+14,&ext));
 assert(r.machine.state==PROTO_READY);{uint8_t event;assert(robot_get_pending_event(&r,&event)&&event==EVENT_TRAJECTORY_DONE);assert(!robot_get_pending_event(&r,&event));}
 now+=15;
 robot_request_stop(&r);assert(robot_control_step(&r,now++,&ext)&&!r.machine.motion_permission&&!r.stop_result.physical_stop_confirmed);
 assert(robot_control_step(&r,600,&ext)==false&&robot_get_state(&r)==PROTO_FAULT);
 puts("robot: PASS");return 0;}

