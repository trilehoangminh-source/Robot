#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "kalman.h"
#include "interpolation.h"
/* Independent double-precision reference using generic 2x2 matrix products. */
static void reference(double x[2],double p[2][2],double dt,double z,double sa,double r){
 double a[2][2]={{1,dt},{0,1}},t[2][2]={{0}},pp[2][2]={{0}};
 double m[2][2],mp[2][2]={{0}},pn[2][2]={{0}},gain[2],innovation;
 int i,j,l;
 x[0]+=dt*x[1];
 for(i=0;i<2;i++)for(j=0;j<2;j++)for(l=0;l<2;l++)t[i][j]+=a[i][l]*p[l][j];
 for(i=0;i<2;i++)for(j=0;j<2;j++)for(l=0;l<2;l++)pp[i][j]+=t[i][l]*a[j][l];
 pp[0][0]+=sa*dt*dt*dt/3;pp[0][1]+=sa*dt*dt/2;pp[1][0]+=sa*dt*dt/2;pp[1][1]+=sa*dt;
 gain[0]=pp[0][0]/(pp[0][0]+r);gain[1]=pp[1][0]/(pp[0][0]+r);
 innovation=z-x[0];for(i=0;i<2;i++)x[i]+=gain[i]*innovation;
 m[0][0]=1-gain[0];m[0][1]=0;m[1][0]=-gain[1];m[1][1]=1;
 for(i=0;i<2;i++)for(j=0;j<2;j++)for(l=0;l<2;l++)mp[i][j]+=m[i][l]*pp[l][j];
 for(i=0;i<2;i++)for(j=0;j<2;j++){
 for(l=0;l<2;l++)pn[i][j]+=mp[i][l]*m[j][l];pn[i][j]+=gain[i]*gain[j]*r;p[i][j]=pn[i][j];}
}
int main(void){
 robot_pid_t pid={0},other={0},saved_pid;pid_config_t pc={2,0,0,0.001f,-1,1,-0.5f,0.5f};
 kalman_config_t kc={0.1f,0.0001f,1,0,1,0};kalman_t k={0},saved,second={0};
 float u,q,z,previous_z=0;int i;double x[2]={0,0},p[2][2]={{1,0},{0,1}};
 double err_k=0,err_d=0;float truth=0;unsigned rng=1;
 assert(pid_initialize(&pid,&pc));assert(pid_update(&pid,0.2f,0,0,&u)&&fabsf(u-0.4f)<1e-6f);
 assert(pid_update(&pid,10,0,0,&u)&&u==1);assert(pid_update(&pid,-10,0,0,&u)&&u==-1);
 pc.kp=0;pc.ki=10;assert(pid_configure(&pid,&pc));assert(pid_update(&pid,1,0,0,&u)&&fabsf(u-0.01f)<1e-6f);
 assert(pid_reset(&pid)&&pid.integral==0);pc.ki=0;pc.kd=0.5f;assert(pid_configure(&pid,&pc));
 assert(pid_update(&pid,0,0,1,&u)&&u==-0.5f);assert(pid_update(&pid,100,0,1,&u)&&u==-0.5f);
 pc.kd=0;pc.kp=1;pc.ki=1;assert(pid_configure(&pid,&pc));
 assert(pid_update(&pid,10,0,0,&u)&&pid.integral==0);
 pid.integral=0.4f;assert(pid_update(&pid,-0.1f,0,0,&u)&&pid.integral<0.4f);
 assert(pid_initialize(&other,&pc)&&other.integral==0);saved_pid=pid;
 assert(!pid_update(&pid,NAN,0,0,&u)&&memcmp(&saved_pid,&pid,sizeof pid)==0);
 pc.dt=0;assert(!pid_configure(&pid,&pc));
 assert(kalman_initialize(&k,&kc));assert(!kalman_predict(&k,0.001f));
 assert(kalman_correct(&k,2)&&k.q==2);assert(kalman_reset(&k,0));
 assert(kalman_initialize(&second,&kc)&&kalman_reset(&second,9));
 for(i=0;i<10000;i++){
 float vel=i<4000?1.0f:(i<7000?0.3f:-0.7f);
 truth+=vel*0.001f;rng=1664525U*rng+1013904223U;
 z=truth+((float)(rng>>8)/16777216.0f-0.5f)*0.02f;z=roundf(z*1000)/1000;
 assert(kalman_update(&k,0.001f,z));assert(kalman_covariance_valid(k.p00,k.p01,k.p11));
 reference(x,p,0.001,z,(double)kc.sa,(double)kc.r);
 assert(fabs((double)k.q-x[0])<0.0001 && fabs((double)k.w-x[1])<0.001);
 if(i>500){double e=k.w-vel,d=(z-previous_z)/0.001-vel;err_k+=e*e;err_d+=d*d;}
 previous_z=z;
 }
 printf("Kalman velocity RMSE=%.6f rad/s; encoder difference RMSE=%.6f rad/s (9499 samples)\n",sqrt(err_k/9499),sqrt(err_d/9499));
 assert(err_k<err_d);assert(second.q==9);saved=k;
 assert(!kalman_update(&k,0,NAN)&&memcmp(&saved,&k,sizeof k)==0);
 assert(!kalman_predict(&k,FLT_MAX)&&memcmp(&saved,&k,sizeof k)==0);
 assert(!kalman_covariance_valid(1,2,1));assert(kalman_predict(&k,0.001f));
 assert(kalman_reset(&k,3)&&k.q==3&&k.w==0);
 for(i=0;i<1000;i++)assert(kalman_update(&k,0.001f,3));assert(fabsf(k.w)<1e-6f);
 assert(interpolation_linear(10,30,1,3,10,&q)==INTERPOLATION_OK&&q==1);
 assert(interpolation_linear(10,30,1,3,20,&q)==INTERPOLATION_OK&&q==2);
 assert(interpolation_linear(10,30,1,3,30,&q)==INTERPOLATION_OK&&q==3);
 assert(interpolation_linear(30,90,3,9,60,&q)==INTERPOLATION_OK&&q==6);
 assert(interpolation_linear(10,30,1,3,0,&q)==INTERPOLATION_OK&&q==1);
 assert(interpolation_linear(10,30,1,3,100,&q)==INTERPOLATION_OK&&q==3);
 assert(interpolation_linear(30,10,1,3,20,&q)==INTERPOLATION_INVALID);
 assert(interpolation_linear(10,10,1,3,20,&q)==INTERPOLATION_INVALID);
 puts("algorithm: PASS");return 0;
}
