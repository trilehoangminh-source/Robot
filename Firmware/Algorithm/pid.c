#include "pid.h"
#include <math.h>
static float clamp(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static bool valid(const pid_config_t *c){
 return c && isfinite(c->kp)&&isfinite(c->ki)&&isfinite(c->kd)&&
 isfinite(c->dt)&&c->dt>0&&isfinite(c->out_min)&&isfinite(c->out_max)&&c->out_min<c->out_max&&
 isfinite(c->i_min)&&isfinite(c->i_max)&&c->i_min<=0&&c->i_max>=0;
}
bool pid_configure(robot_pid_t *p,const pid_config_t *c){
 if(!p||!valid(c))return false;
 p->config=*c;p->integral=0;p->configured=true;return true;
}
bool pid_initialize(robot_pid_t *p,const pid_config_t *c){return pid_configure(p,c);}
bool pid_reset(robot_pid_t *p){if(!p||!p->configured)return false;p->integral=0;return true;}
bool pid_update(robot_pid_t *p,float r,float q,float v,float *out){
 float e,base,increment,candidate,u;pid_config_t c;
 if(!p||!out||!p->configured||!valid(&p->config)||!isfinite(r)||!isfinite(q)||!isfinite(v)||!isfinite(p->integral))return false;
 c=p->config;e=r-q;base=c.kp*e-c.kd*v;increment=c.ki*e*c.dt;
 if(!isfinite(e)||!isfinite(base)||!isfinite(increment))return false;
 candidate=clamp(p->integral+increment,c.i_min,c.i_max);u=base+candidate;
 if(!isfinite(u))return false;
 if((u>c.out_max&&increment>0)||(u<c.out_min&&increment<0))candidate=p->integral;
 u=base+candidate;if(!isfinite(u))return false;
 p->integral=candidate;*out=clamp(u,c.out_min,c.out_max);return true;
}
