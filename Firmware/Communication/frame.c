#include "frame.h"
#include "crc.h"
#include <string.h>
frame_status_t frame_encode(const frame_t *f,uint8_t *out,size_t cap,size_t *written){size_t n;if(!f||!out||!written)return FRAME_ARGUMENT;*written=0;n=(size_t)f->length+8;if(f->length>128||cap<n)return FRAME_LENGTH;out[0]=COMM_SOF_0;out[1]=COMM_SOF_1;out[2]=COMM_PROTOCOL_VERSION;out[3]=f->id;out[4]=f->seq;out[5]=f->length;memcpy(out+6,f->payload,f->length);protocol_put_u16(out+6+f->length,crc16_ccitt_false(out+2,4+f->length));*written=n;return FRAME_OK;}
frame_status_t frame_decode(const uint8_t *d,size_t n,frame_t *out){frame_t f;if(!d||!out)return FRAME_ARGUMENT;if(n<8)return FRAME_LENGTH;if(d[0]!=COMM_SOF_0||d[1]!=COMM_SOF_1)return FRAME_SOF;if(d[5]>128||n!=(size_t)d[5]+8)return FRAME_LENGTH;if(d[2]!=COMM_PROTOCOL_VERSION)return FRAME_VERSION;if(protocol_u16(d+n-2)!=crc16_ccitt_false(d+2,n-4))return FRAME_CRC;f.id=d[3];f.seq=d[4];f.length=d[5];memcpy(f.payload,d+6,f.length);*out=f;return FRAME_OK;}
