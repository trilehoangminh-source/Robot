#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "trajectory_controller.h"
int main(void){trajectory_controller_t t;trajectory_config_t c={0};float q[3]={0},w[3]={0},out[3];unsigned i;
 trajectory_initialize(&t);assert(!trajectory_begin(&t,1,2));c.verified=true;c.settle_ms=5;
 for(i=0;i<3;i++){c.min_q[i]=-10;c.max_q[i]=10;c.max_velocity[i]=2;c.max_acceleration[i]=2000;c.start_tolerance[i]=0.001f;c.complete_error[i]=0.001f;c.complete_velocity[i]=0.01f;}
 assert(trajectory_configure(&t,&c));assert(!trajectory_begin(&t,1,129));assert(trajectory_begin(&t,0,3));
 assert(!trajectory_append(&t,1,0,0,q));assert(!trajectory_append(&t,0,1,0,q));assert(!trajectory_append(&t,0,0,1,q));
 assert(trajectory_append(&t,0,0,0,q));assert(!trajectory_start(&t,0,0,q));
 q[0]=0.1f;assert(trajectory_append(&t,0,1,100,q));q[0]=0.3f;assert(trajectory_append(&t,0,2,300,q));q[0]=0;
 assert(trajectory_start(&t,0,UINT32_MAX-49,q));assert(!trajectory_begin(&t,1,1));
 assert(trajectory_update(&t,0,q,w,out)&&fabsf(out[0]-0.05f)<1e-5f);
 assert(trajectory_update(&t,100,q,w,out)&&fabsf(out[0]-0.15f)<1e-5f);
 assert(trajectory_update(&t,250,q,w,out)&&t.state==TRAJ_SETTLING);
 assert(trajectory_update(&t,260,q,w,out)&&t.state!=TRAJ_DONE);
 q[0]=0.3f;assert(trajectory_update(&t,261,q,w,out));assert(trajectory_update(&t,266,q,w,out)&&t.state==TRAJ_DONE);
 trajectory_abort(&t);assert(!trajectory_update(&t,267,q,w,out));assert(trajectory_clear(&t));
 assert(trajectory_begin(&t,2,128));for(i=0;i<128;i++)assert(trajectory_append(&t,2,(uint16_t)i,i,q));assert(!trajectory_append(&t,2,128,128,q));
 assert(trajectory_begin(&t,2,2));q[0]=NAN;assert(!trajectory_append(&t,2,0,0,q));q[0]=INFINITY;assert(!trajectory_append(&t,2,0,0,q));
 q[0]=11;assert(!trajectory_append(&t,2,0,0,q));q[0]=0;assert(trajectory_append(&t,2,0,0,q));assert(!trajectory_append(&t,2,1,0,q));q[0]=1;assert(trajectory_append(&t,2,1,1,q));q[0]=0;assert(!trajectory_validate(&t,q));
 puts("trajectory_controller: PASS");return 0;}
