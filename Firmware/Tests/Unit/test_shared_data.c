#include <assert.h>
#include <stdio.h>
#include "control_task.h"
int main(void){command_t c={0},out;frame_t f={0},response;shared_snapshot_t snapshot={0},copy;unsigned i;uint32_t now;
 shared_data_initialize();c.id=COMM_CMD_HOME;
 for(i=0;i<SHARED_COMMAND_CAPACITY;i++)assert(shared_command_push(&c));assert(!shared_command_push(&c));
 shared_stop_request();assert(!shared_command_push(&c));assert(!shared_command_pop(&out));assert(shared_stop_take());assert(!shared_command_pop(&out));
 assert(shared_command_push(&c)&&shared_command_pop(&out)&&out.id==COMM_CMD_HOME);
 shared_heartbeat_publish(UINT32_MAX);assert(shared_heartbeat_take(&now)&&now==UINT32_MAX);assert(!shared_heartbeat_take(&now));
 for(i=0;i<SHARED_RESPONSE_CAPACITY;i++)assert(shared_response_push(&f));assert(!shared_response_push(&f)&&shared_response_dropped()==1);assert(shared_response_pop(&response));
 assert(!shared_snapshot_read(&copy));snapshot.steps=42;snapshot.state=PROTO_FAULT;snapshot.latched_faults=FAULT_DEADLINE;shared_snapshot_publish(&snapshot);assert(shared_snapshot_read(&copy)&&copy.steps==42&&copy.state==PROTO_FAULT&&copy.latched_faults==FAULT_DEADLINE);
 assert(!control_task_deadline(1,999,1000));assert(control_task_deadline(2,100,1000));assert(control_task_deadline(0,0,1000));assert(control_task_deadline(1,1000,1000));
 printf("shared_data: PASS; snapshot=%zu bytes, command rings=%zu bytes, response rings=%zu bytes\n",sizeof(snapshot),sizeof(command_t)*SHARED_COMMAND_CAPACITY,sizeof(frame_t)*SHARED_RESPONSE_CAPACITY);return 0;}
