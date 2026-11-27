#include "trajectory_controller.h"
#include "interpolation.h"
#include <math.h>
void trajectory_initialize(trajectory_controller_t *t){trajectory_controller_t z={0};if(t)*t=z;}
static bool running(const trajectory_controller_t *t){return t->state==TRAJ_RUNNING||t->state==TRAJ_SETTLING;}
bool trajectory_configure(trajectory_controller_t *t,const trajectory_config_t *c){unsigned j;if(!t||!c||!c->verified||running(t)||!c->settle_ms||c->settle_ms>0x7fffffffU)return false;
 for(j=0;j<3;j++)if(!isfinite(c->min_q[j])||!isfinite(c->max_q[j])||c->min_q[j]>=c->max_q[j]||!isfinite(c->max_velocity[j])||c->max_velocity[j]<=0||!isfinite(c->max_acceleration[j])||c->max_acceleration[j]<=0||!isfinite(c->start_tolerance[j])||c->start_tolerance[j]<0||!isfinite(c->complete_error[j])||c->complete_error[j]<0||!isfinite(c->complete_velocity[j])||c->complete_velocity[j]<0)return false;
 t->config=*c;t->configured=true;return true;}
bool trajectory_begin(trajectory_controller_t *t,uint16_t id,uint16_t count){if(!t||!t->configured||running(t)||!count||count>COMM_TRAJECTORY_CAPACITY_POINTS)return false;t->id=id;t->expected=count;t->received=0;t->segment=0;t->stable=false;t->clock_valid=false;t->state=TRAJ_LOADING;return true;}
bool trajectory_append(trajectory_controller_t *t,uint16_t id,uint16_t index,uint32_t ms,const float q[3]){unsigned j;if(!t||!q||t->state!=TRAJ_LOADING||id!=t->id||index!=t->received||index>=t->expected||ms>0x7fffffffU||(index==0&&ms!=0)||(index&&ms<=t->points[index-1].time_ms))return false;
 for(j=0;j<3;j++)if(!isfinite(q[j])||q[j]<t->config.min_q[j]||q[j]>t->config.max_q[j])return false;
 t->points[index].time_ms=ms;for(j=0;j<3;j++)t->points[index].q[j]=q[j];t->received++;return true;}
bool trajectory_validate(const trajectory_controller_t *t,const float current[3]){unsigned j,i;if(!t||!current||!t->configured||t->state!=TRAJ_LOADING||t->received!=t->expected||!t->received)return false;
 for(j=0;j<3;j++){float previous=0;if(!isfinite(current[j])||fabsf(current[j]-t->points[0].q[j])>t->config.start_tolerance[j])return false;
 for(i=1;i<t->received;i++){float dt=(float)(t->points[i].time_ms-t->points[i-1].time_ms)*0.001f;float v=(t->points[i].q[j]-t->points[i-1].q[j])/dt;
 if(!isfinite(v)||fabsf(v)>t->config.max_velocity[j]||fabsf(v-previous)/0.001f>t->config.max_acceleration[j])return false;previous=v;}
 if(fabsf(previous)/0.001f>t->config.max_acceleration[j])return false;}
 return true;}
bool trajectory_start(trajectory_controller_t *t,uint16_t id,uint32_t now,const float current[3]){if(!t||id!=t->id||!trajectory_validate(t,current))return false;t->start_ms=now;t->last_ms=now;t->clock_valid=true;t->state=TRAJ_RUNNING;t->stable=false;return true;}
bool trajectory_update(trajectory_controller_t *t,uint32_t now,const float q[3],const float w[3],float out[3]){uint32_t elapsed;unsigned j;bool settled=true;if(!t||!q||!w||!out||(!running(t)&&t->state!=TRAJ_DONE))return false;
 if(t->clock_valid&&(uint32_t)(now-t->last_ms)>0x7fffffffU)return false;t->last_ms=now;
 elapsed=now-t->start_ms;
 for(j=0;j<3;j++)if(!isfinite(q[j])||!isfinite(w[j]))return false;
 while(t->segment+1U<t->received&&elapsed>=t->points[t->segment+1U].time_ms)t->segment++;
 for(j=0;j<3;j++){
 if(t->segment+1U<t->received){if(interpolation_linear(t->points[t->segment].time_ms,t->points[t->segment+1U].time_ms,t->points[t->segment].q[j],t->points[t->segment+1U].q[j],elapsed,&out[j])!=INTERPOLATION_OK)return false;}
 else out[j]=t->points[t->received-1U].q[j];
 if(fabsf(out[j]-q[j])>t->config.complete_error[j]||fabsf(w[j])>t->config.complete_velocity[j])settled=false;
 }
 if(elapsed>=t->points[t->received-1U].time_ms&&t->state!=TRAJ_DONE){t->state=TRAJ_SETTLING;if(settled){if(!t->stable){t->stable=true;t->stable_ms=now;}if((uint32_t)(now-t->stable_ms)>=t->config.settle_ms)t->state=TRAJ_DONE;}else t->stable=false;}
 return true;}
void trajectory_abort(trajectory_controller_t *t){if(t){t->state=TRAJ_ABORTED;t->received=0;t->stable=false;}}
bool trajectory_clear(trajectory_controller_t *t){if(!t||running(t))return false;t->state=TRAJ_EMPTY;t->received=0;return true;}
trajectory_state_t trajectory_get_status(const trajectory_controller_t *t){return t?t->state:TRAJ_ABORTED;}
