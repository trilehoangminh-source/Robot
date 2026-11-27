#include "motor.h"
#include <math.h>
static mdd3a_t drivers[3];
static int signs[3];
static float limits[3];
bsp_status_t motor_configure(uint8_t j,int sign,float max,bool verified) {
 if(j>=3 || (sign!=1 && sign!=-1) || !(max>0 && max<=1))return BSP_INVALID_ARGUMENT;
 if(drivers[j].enabled)return BSP_BUSY;
 signs[j]=0; if(!verified)return BSP_NOT_READY;
 signs[j]=sign; limits[j]=max; return BSP_OK;
}
bsp_status_t motor_enable(uint8_t j) {
 if(j>=3)return BSP_INVALID_ARGUMENT;
 if(!signs[j])return BSP_NOT_READY; return mdd3a_enable(&drivers[j],j);
}
bsp_status_t motor_disable(uint8_t j) {
 if(j>=3)return BSP_INVALID_ARGUMENT; return mdd3a_disable(&drivers[j],j);
}
bsp_status_t motor_write(uint8_t j,float u) {
 if(j>=3 || !isfinite(u))return BSP_INVALID_ARGUMENT;
 if(!signs[j])return BSP_NOT_READY;
 if(u>limits[j])u=limits[j]; if(u< -limits[j])u=-limits[j];
 return mdd3a_write(&drivers[j],j,u*(float)signs[j]);
}
