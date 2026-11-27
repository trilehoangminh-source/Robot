#include "gripper.h"
bool gripper_initialize(gripper_t *g,const platform_t *p){gripper_t z={0};if(!g||!p||!p->ops)return false;*g=z;g->platform=*p;g->command=2;return true;}
bool gripper_configure(gripper_t *g,bool verified){if(!g)return false;g->configured=verified;return verified;}
platform_status_t gripper_request(gripper_t *g,bool close,bool permission){platform_status_t s;if(!g)return PLATFORM_INVALID_ARGUMENT;g->accepted=false;if(!g->configured||!permission||g->disable_requested)return PLATFORM_NOT_READY;
 s=platform_set_gripper(&g->platform,close?PLATFORM_GRIPPER_CLOSE:PLATFORM_GRIPPER_OPEN);
 if(s==PLATFORM_OK){g->accepted=true;g->command=close?1U:0U;}return s;}
void gripper_abort(gripper_t *g){if(g){g->accepted=false;g->disable_requested=true;g->command=2;}}
const gripper_t *gripper_get_status(const gripper_t *g){return g;}
