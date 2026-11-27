#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "robot_task.h"
#include "telemetry_task.h"
#include "communication_task.h"
static unsigned writes;static frame_t transmitted;
static bsp_status_t tx(const uint8_t *p,uint16_t n){assert(frame_decode(p,n,&transmitted)==FRAME_OK);writes++;return BSP_OK;}
static bsp_status_t ok(void){return BSP_OK;}
static void feed(communication_task_t *t,uint8_t id,uint8_t seq,uint32_t now){frame_t f={0};uint8_t b[136];size_t n;f.id=id;f.seq=seq;assert(frame_encode(&f,b,sizeof b,&n)==FRAME_OK);assert(communication_feed(t,b,n,now));}
int main(void){robot_task_t robot;telemetry_task_t tele;communication_task_t comm;communication_io_t io={tx,ok,ok};shared_snapshot_t s={0},copy;shared_application_diagnostics_t a;shared_telemetry_diagnostics_t d;frame_t f={0};command_t c={0};uint32_t stamp;unsigned i;
 shared_data_initialize();shared_snapshot_publish(&s);robot_task_initialize(&robot,UINT32_MAX-5U);assert(!robot_task_step(&robot,3));assert(robot_task_step(&robot,4));assert(!robot_task_step(&robot,4));assert(robot_task_step(&robot,39));assert(robot.missed_periods==2&&robot.pending_requests==1&&robot.next_ms==44);shared_application_diagnostics(&a);assert(a.pending&&a.requested==2&&a.pending_collisions==1);assert(shared_application_take(&stamp)&&stamp==4);assert(!shared_application_take(&stamp));assert(!robot_task_step(&robot,40));
 shared_data_initialize();assert(communication_initialize(&comm,&io));feed(&comm,COMM_CMD_PING,1,0);assert(!comm.pending&&!shared_command_pop(&c));communication_poll(&comm,0,false,false,false);assert(transmitted.id==COMM_MSG_PONG&&transmitted.seq==1);
 feed(&comm,COMM_CMD_GET_INFO,2,1);communication_poll(&comm,1,false,false,false);assert(transmitted.id==COMM_MSG_ROBOT_INFO&&transmitted.seq==2);
 feed(&comm,COMM_CMD_ENABLE,3,2);assert(!comm.pending&&!shared_command_pop(&c));communication_poll(&comm,2,false,false,false);assert(transmitted.id==COMM_MSG_NACK&&transmitted.payload[1]==NACK_BUSY);
 feed(&comm,COMM_CMD_STOP,4,3);assert(!comm.stop_pending&&shared_stop_take());communication_poll(&comm,3,false,false,false);assert(transmitted.id==COMM_MSG_NACK&&transmitted.seq==4);
 robot_task_initialize(&robot,0);assert(robot_task_step(&robot,10)&&robot.startup_blocked==1);assert(!shared_application_take(&stamp));
 telemetry_task_initialize(&tele,0);telemetry_task_step(&tele,20,true);assert(tele.startup_blocked==1&&!shared_telemetry_pop(&f));
 s.telemetry_valid=true;s.debug_valid=true;s.telemetry.feedback_valid=true;s.telemetry.gripper=2;s.telemetry.time_ms=s.published_ms=40;s.telemetry.q[0]=1.25f;s.telemetry.w[2]=-0.5f;s.telemetry.reference[0]=2;s.telemetry.error[0]=0.75f;s.telemetry.output[0]=0.3f;shared_snapshot_publish(&s);
 assert(shared_snapshot_read(&copy));s.telemetry.q[0]=9;assert(copy.telemetry.q[0]==1.25f);s=copy;
 telemetry_task_step(&tele,40,true);assert(tele.generated==2&&tele.enqueued==2);assert(shared_telemetry_pop(&f)&&f.id==COMM_MSG_JOINT_STATE&&f.length==34);{float value;assert(protocol_float(f.payload+4,&value)&&value==1.25f);}assert(shared_telemetry_pop(&f)&&f.id==COMM_MSG_CONTROL_DEBUG&&f.length==40);{float value;assert(protocol_float(f.payload+16,&value)&&value==0.75f);}
 telemetry_task_step(&tele,40,true);assert(tele.generated==2);
 s.telemetry.q[0]=NAN;s.published_ms=s.telemetry.time_ms=60;shared_snapshot_publish(&s);telemetry_task_step(&tele,60,false);assert(tele.invalid==1);
 s.telemetry.q[0]=1.25f;s.published_ms=s.telemetry.time_ms=80;shared_snapshot_publish(&s);telemetry_task_step(&tele,80,true);telemetry_task_step(&tele,100,true);shared_telemetry_diagnostics(&d);assert(d.replaced==2&&d.completed==0);
 telemetry_task_step(&tele,140,true);assert(tele.stale==1&&tele.missed_periods==1);s.telemetry_valid=false;s.published_ms=s.telemetry.time_ms=160;shared_snapshot_publish(&s);telemetry_task_step(&tele,160,true);assert(tele.invalid==2);
 s.telemetry_valid=true;s.published_ms=s.telemetry.time_ms=180;s.telemetry.output[0]=INFINITY;shared_snapshot_publish(&s);telemetry_task_step(&tele,180,true);assert(tele.invalid==3);s.telemetry.output[0]=0.3f;
 telemetry_task_initialize(&tele,UINT32_MAX-9U);s.published_ms=s.telemetry.time_ms=10;shared_snapshot_publish(&s);telemetry_task_step(&tele,10,false);assert(tele.generated==1);
 /* Unsolicited sequence belongs to Comm, high priority precedes telemetry. */
 shared_data_initialize();s.telemetry_valid=true;s.published_ms=s.telemetry.time_ms=200;shared_snapshot_publish(&s);assert(communication_initialize(&comm,&io));telemetry_task_initialize(&tele,180);telemetry_task_step(&tele,200,true);f.id=COMM_MSG_ACK;f.seq=77;f.length=2;assert(communication_transmit(&comm,&f));communication_poll(&comm,200,false,false,false);assert(transmitted.id==COMM_MSG_ACK&&transmitted.seq==77);communication_poll(&comm,201,false,false,false);assert(transmitted.id==COMM_MSG_JOINT_STATE);shared_telemetry_diagnostics(&d);assert(d.completed==0);communication_poll(&comm,202,false,false,false);assert(transmitted.id==COMM_MSG_CONTROL_DEBUG&&transmitted.seq==1);shared_telemetry_diagnostics(&d);assert(d.completed==1);communication_poll(&comm,203,false,false,false);shared_telemetry_diagnostics(&d);assert(d.completed==2);
 /* Stale telemetry is discarded after waiting behind UART traffic. */
 f.id=COMM_MSG_JOINT_STATE;f.length=34;protocol_put_u32(f.payload,200);assert(shared_telemetry_push(&f));communication_poll(&comm,250,false,false,false);shared_telemetry_diagnostics(&d);assert(d.transport_dropped==1&&d.completed==2);
 /* STOP and heartbeat remain independent even when both regular queues fill. */
 f.id=COMM_MSG_ACK;for(i=0;i<COMM_TX_HIGH_CAPACITY;i++)assert(communication_transmit(&comm,&f));c.id=COMM_CMD_HOME;for(i=0;i<SHARED_COMMAND_CAPACITY;i++)assert(shared_command_push(&c));feed(&comm,COMM_CMD_HEARTBEAT,80,210);assert(shared_heartbeat_take(&stamp)&&stamp==210);feed(&comm,COMM_CMD_STOP,81,211);assert(shared_stop_command_take(&c)&&c.seq==81&&!shared_command_pop(&c));
 /* A valid pending request is canceled if the owner reports startup blocked. */
 shared_data_initialize();s.startup_blocked=false;shared_snapshot_publish(&s);assert(communication_initialize(&comm,&io));feed(&comm,COMM_CMD_ENABLE,9,220);assert(comm.pending);s.startup_blocked=true;shared_snapshot_publish(&s);communication_poll(&comm,221,false,false,false);assert(!comm.pending&&comm.response.id==COMM_MSG_NACK&&!shared_command_pop(&c));
 printf("8.3 pipeline: PASS; robot=%zu telemetry=%zu snapshot=%zu bytes\n",sizeof robot,sizeof tele,sizeof s);return 0;}
