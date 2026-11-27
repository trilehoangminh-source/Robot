#include "crc.h"
uint16_t crc16_ccitt_false(const uint8_t *p,size_t n){uint16_t c=COMM_CRC16_INITIAL;size_t i;unsigned b;if(!p&&n)return 0;for(i=0;i<n;i++){c^=(uint16_t)((uint16_t)p[i]<<8);for(b=0;b<8;b++)c=(uint16_t)((c&0x8000U)?((uint32_t)c<<1)^COMM_CRC16_POLYNOMIAL:(uint32_t)c<<1);}return c;}
