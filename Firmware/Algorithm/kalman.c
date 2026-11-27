#include "kalman.h"
#include <math.h>
bool kalman_covariance_valid(float a,float b,float d){
 float scale;
 if(!isfinite(a)||!isfinite(b)||!isfinite(d)||a<0||d<0)return false;
 scale=fmaxf(a,fmaxf(fabsf(b),d));
 if(scale==0)return true;
 return (a/scale)*(d/scale)>=(b/scale)*(b/scale);
}
static bool config_valid(const kalman_config_t *c){return c&&isfinite(c->sa)&&c->sa>=0&&isfinite(c->r)&&c->r>0&&isfinite(c->initial_velocity)&&kalman_covariance_valid(c->p00,c->p01,c->p11);}
static bool state_valid(const kalman_t *k){return k&&k->configured&&k->initialized&&config_valid(&k->config)&&isfinite(k->q)&&isfinite(k->w)&&kalman_covariance_valid(k->p00,k->p01,k->p11);}
bool kalman_configure(kalman_t *k,const kalman_config_t *c){
 kalman_t fresh={0};if(!k||!config_valid(c))return false;
 fresh.config=*c;fresh.configured=true;*k=fresh;return true;
}
bool kalman_initialize(kalman_t *k,const kalman_config_t *c){return kalman_configure(k,c);}
bool kalman_reset(kalman_t *k,float q){
 if(!k||!k->configured||!config_valid(&k->config)||!isfinite(q))return false;
 k->q=q;k->w=k->config.initial_velocity;k->p00=k->config.p00;k->p01=k->config.p01;k->p11=k->config.p11;k->initialized=true;return true;
}
bool kalman_predict(kalman_t *k,float dt){
 kalman_t n;float t2;
 if(!state_valid(k)||!isfinite(dt)||dt<=0)return false;
 n=*k;t2=dt*dt;
 n.q=k->q+dt*k->w;
 n.p00=k->p00+2*dt*k->p01+t2*k->p11+k->config.sa*t2*dt/3;
 n.p01=k->p01+dt*k->p11+k->config.sa*t2/2;
 n.p11=k->p11+k->config.sa*dt;
 if(!state_valid(&n))return false;*k=n;return true;
}
bool kalman_correct(kalman_t *k,float z){
 kalman_t n;float s,k0,k1,a,b,d,h,innovation,r;
 if(!k||!k->configured||!isfinite(z))return false;
 if(!k->initialized)return kalman_reset(k,z);
 if(!state_valid(k))return false;
 s=k->p00+k->config.r;if(!isfinite(s)||s<=0)return false;
 k0=k->p00/s;k1=k->p01/s;innovation=z-k->q;
 n=*k;n.q=k->q+k0*innovation;n.w=k->w+k1*innovation;
 /* Joseph: (I-KH)P(I-KH)' + KRK', symmetric storage. */
 a=k->p00;b=k->p01;d=k->p11;h=1-k0;r=k->config.r;
 n.p00=h*h*a+k0*k0*r;
 n.p01=h*(b-k1*a)+k0*k1*r;
 n.p11=d-2*k1*b+k1*k1*a+k1*k1*r;
 if(!isfinite(innovation)||!state_valid(&n))return false;*k=n;return true;
}
bool kalman_update(kalman_t *k,float dt,float z){
 kalman_t n;
 if(!k||!isfinite(dt)||dt<=0||!isfinite(z))return false;
 n=*k;if(n.initialized&&!kalman_predict(&n,dt))return false;
 if(!kalman_correct(&n,z))return false;*k=n;return true;
}
