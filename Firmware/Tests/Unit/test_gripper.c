#include <assert.h>
#include <stdio.h>
#include "gripper.h"
int main(void){platform_t p;platform_sim_state_t sim={0};gripper_t g;
 assert(platform_sim_bind(&p,&sim)==PLATFORM_OK);assert(gripper_initialize(&g,&p)&&g.command==2);
 assert(gripper_request(&g,false,true)==PLATFORM_NOT_READY);assert(gripper_configure(&g,true));
 assert(gripper_request(&g,false,false)==PLATFORM_NOT_READY);sim.ready=true;
 assert(gripper_request(&g,false,true)==PLATFORM_OK&&g.command==0&&!g.mechanical_known&&!g.output_known);
 assert(gripper_request(&g,true,true)==PLATFORM_OK&&g.command==1);gripper_abort(&g);
 assert(gripper_request(&g,true,true)==PLATFORM_NOT_READY&&g.disable_requested);
 puts("gripper: PASS");return 0;}
