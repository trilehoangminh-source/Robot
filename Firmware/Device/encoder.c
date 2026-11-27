#include "encoder.h"
#include <math.h>
#include <limits.h>
static robot_joint_calibration_t cfg[3];
static int64_t counts[3];
static bool started[3],cached[3];
static uint32_t last_step[3],epoch[3];
static platform_encoder_feedback_t snapshot[3];
static bool valid_config(const robot_joint_calibration_t *c) {
 return c && c->calibration_valid && c->encoder_cpr &&
 (c->encoder_sign==1 || c->encoder_sign==-1) && isfinite(c->home_angle_rad);
}
bsp_status_t encoder_convert(int64_t count,int32_t delta,uint32_t dt,
 const robot_joint_calibration_t *c,float *q,float *v) {
 double scale;
 if(!q || !v || !valid_config(c) || !dt)return BSP_INVALID_ARGUMENT;
 scale=6.283185307179586*(double)c->encoder_sign/c->encoder_cpr;
 *q=(float)(c->home_angle_rad+scale*((double)count-c->home_count));
 *v=(float)(scale*delta*1000.0/dt);
 return isfinite(*q)&&isfinite(*v)?BSP_OK:BSP_IO_ERROR;
}
bsp_status_t encoder_configure(uint8_t j,const robot_joint_calibration_t *c) {
 if(j>=3 || !valid_config(c))return BSP_INVALID_ARGUMENT;
 if(started[j])return BSP_BUSY;
 cfg[j]=*c; counts[j]=c->home_count; ++epoch[j]; cached[j]=false; return BSP_OK;
}
bsp_status_t encoder_start(uint8_t j) {
 if(j>=3)return BSP_INVALID_ARGUMENT;
 if(!valid_config(&cfg[j]))return BSP_NOT_READY;
 {bsp_status_t s=bsp_encoder_start(j); if(s==BSP_OK)started[j]=true; return s;}
}
bsp_status_t encoder_update(uint8_t j,uint32_t dt,encoder_feedback_t *f) {
 int32_t delta; bsp_status_t s;
 if(!f)return BSP_INVALID_ARGUMENT; f->valid=false;
 if(j>=3 || !dt)return BSP_INVALID_ARGUMENT;
 if(!valid_config(&cfg[j]))return BSP_NOT_READY;
 s=bsp_encoder_sample_delta(j,&delta); if(s!=BSP_OK)return s;
 if((delta>0 && counts[j]>INT64_MAX-delta)||(delta<0 && counts[j]<INT64_MIN-delta))return BSP_IO_ERROR;
 counts[j]+=delta; f->count=counts[j];
 s=encoder_convert(counts[j],delta,dt,&cfg[j],&f->position_rad,&f->velocity_rad_s);
 f->valid=s==BSP_OK; return s;
}
bsp_status_t encoder_reset_reference(uint8_t j,float angle) {
 bsp_status_t s;
 if(j>=3 || !isfinite(angle))return BSP_INVALID_ARGUMENT;
 if(!valid_config(&cfg[j]))return BSP_NOT_READY;
 s=bsp_encoder_resync(j); if(s!=BSP_OK)return s;
 ++epoch[j];cached[j]=false;counts[j]=0; cfg[j].home_count=0; cfg[j].home_angle_rad=angle; return BSP_OK;
}

bsp_status_t encoder_sample(uint8_t j,uint32_t step,uint32_t dt,platform_encoder_feedback_t *out){
 encoder_feedback_t f;uint32_t raw;bsp_status_t s;
 if(j>=3||!out)return BSP_INVALID_ARGUMENT;
 if(cached[j]&&last_step[j]==step){*out=snapshot[j];return out->valid?BSP_OK:BSP_IO_ERROR;}
 snapshot[j].valid=false;last_step[j]=step;cached[j]=true;
 s=encoder_update(j,dt,&f);if(s!=BSP_OK){*out=snapshot[j];return s;}
 s=bsp_encoder_read_raw(j,&raw);if(s!=BSP_OK){*out=snapshot[j];return s;}
 snapshot[j].raw=raw;snapshot[j].count=f.count;snapshot[j].q=f.position_rad;
 snapshot[j].w=f.velocity_rad_s;snapshot[j].reference_epoch=epoch[j];snapshot[j].valid=true;
 *out=snapshot[j];return BSP_OK;
}
