#include "protocol.h"
#include <float.h>
#include <math.h>
#include <string.h>
typedef char require_float32[(sizeof(float)==4 && FLT_RADIX==2 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128)?1:-1];
void protocol_put_u16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
void protocol_put_u32(uint8_t *p,uint32_t v){unsigned i;for(i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
uint16_t protocol_u16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
uint32_t protocol_u32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
bool protocol_put_float(uint8_t *p,float v){uint32_t bits;if(!p||!isfinite(v))return false;memcpy(&bits,&v,4);protocol_put_u32(p,bits);return true;}
bool protocol_float(const uint8_t *p,float *v){uint32_t bits;float f;if(!p||!v)return false;bits=protocol_u32(p);memcpy(&f,&bits,4);if(!isfinite(f))return false;*v=f;return true;}
