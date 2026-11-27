#include "homing.h"
#include <math.h>
static void stop(homing_t *h){h->intent.motion=false;h->intent.output=0;h->intent.stop_requested=true;h->intent.reference_request=false;}
static void phase(homing_t *h,homing_phase_t p,uint32_t now){h->phase=p;h->phase_ms=now;stop(h);}
bool homing_initialize(homing_t *h,const homing_config_t *c){homing_t z={0};unsigned i;unsigned mask=0;if(!h||!c)return false;*h=z;
 if(!c->verified||!c->total_timeout_ms||c->total_timeout_ms>0x7fffffffU)return false;
 for(i=0;i<3;i++){const homing_joint_config_t *j=&c->joint[i];if(c->order[i]>2||(mask&(1U<<c->order[i])))return false;mask|=1U<<c->order[i];
 if((j->direction!=1&&j->direction!=-1)||!isfinite(j->home_angle)||!(j->seek>0&&j->seek<=1)||!(j->slow>0&&j->slow<=j->seek)||!(j->backoff>0&&j->backoff<=1)||!isfinite(j->backoff_rad)||j->backoff_rad<=0||!isfinite(j->max_travel)||j->max_travel<=0||!j->phase_timeout_ms||j->phase_timeout_ms>0x7fffffffU)return false;}
 h->config=*c;h->configured=true;return true;}
bool homing_start(homing_t *h,const homing_input_t *in){if(!h||!in||!h->configured||h->phase!=HOME_IDLE||!in->permission||!in->safe||!in->feedback_valid||!isfinite(in->q))return false;
 h->index=0;h->start_ms=in->now_ms;h->start_q=in->q;phase(h,HOME_CHECK,in->now_ms);return true;}
void homing_abort(homing_t *h){if(h){h->homed[0]=h->homed[1]=h->homed[2]=false;phase(h,HOME_ABORTED,h->phase_ms);}}
bool homing_update(homing_t *h,const homing_input_t *in){homing_joint_config_t *c;uint8_t id;float u=0;
 if(!h||!in||!h->configured)return false;
 if(h->phase==HOME_IDLE||h->phase>=HOME_DONE)return false;
 stop(h);id=h->config.order[h->index];h->intent.joint=id;c=&h->config.joint[id];
 if(!in->permission||!in->safe){homing_abort(h);return false;}
 if(!in->feedback_valid||!in->switch_valid||!isfinite(in->q)||in->driver_fault||(uint32_t)(in->now_ms-h->start_ms)>=h->config.total_timeout_ms||(uint32_t)(in->now_ms-h->phase_ms)>=c->phase_timeout_ms||fabsf(in->q-h->start_q)>c->max_travel){phase(h,HOME_FAULT,in->now_ms);return false;}
 switch(h->phase){
 case HOME_CHECK:h->start_q=in->q;phase(h,in->raw_active?HOME_RELEASE:HOME_SEEK,in->now_ms);break;
 case HOME_RELEASE:u=-c->backoff*(float)c->direction;if(!in->raw_active&&!in->debounced_active)phase(h,HOME_SEEK,in->now_ms);break;
 case HOME_SEEK:case HOME_SLOW:
 if(in->raw_active){if(in->debounced_active&&in->stop_confirmed){if(h->phase==HOME_SEEK){h->backoff_q=in->q;phase(h,HOME_BACKOFF,in->now_ms);}else phase(h,HOME_REFERENCE,in->now_ms);}}
 else u=(h->phase==HOME_SEEK?c->seek:c->slow)*(float)c->direction;break;
 case HOME_BACKOFF:u=-c->backoff*(float)c->direction;if(!in->raw_active&&!in->debounced_active&&fabsf(in->q-h->backoff_q)>=c->backoff_rad)phase(h,HOME_SLOW,in->now_ms);break;
 case HOME_REFERENCE:h->intent.reference_request=true;h->intent.home_angle=c->home_angle;
 if(in->stop_confirmed&&in->reference_committed&&in->controller_synchronized){h->homed[id]=true;h->intent.reference_request=false;
 if(++h->index==3)phase(h,HOME_DONE,in->now_ms);else {h->start_q=in->q;phase(h,HOME_CHECK,in->now_ms);}}
 break;
 default:break;
 }
 /* A phase change always emits stop before next phase begins. */
 if(h->phase_ms!=in->now_ms&&u!=0){h->intent.motion=true;h->intent.stop_requested=false;h->intent.output=u;}
 return h->phase!=HOME_FAULT&&h->phase!=HOME_ABORTED;
}
