#include "parser.h"
#include <string.h>
bool parser_initialize(parser_t *p,uint32_t timeout){parser_t empty={0};if(!p||!timeout||timeout>0x7fffffffU)return false;*p=empty;p->timeout_ms=timeout;return true;}
void parser_expire(parser_t *p,uint32_t now){if(p&&p->used&&(uint32_t)(now-p->last_ms)>=p->timeout_ms){p->used=0;p->timeouts++;}}
static void drop(parser_t *p,size_t n){memmove(p->buffer,p->buffer+n,p->used-n);p->used-=n;}
bool parser_feed(parser_t *p,const uint8_t *data,size_t length,uint32_t now,parser_callback_t cb,void *ctx){size_t i,n;frame_t f;frame_status_t s;if(!p||(!data&&length)||!cb||!p->timeout_ms)return false;parser_expire(p,now);for(i=0;i<length;i++){p->last_ms=now;p->buffer[p->used++]=data[i];for(;;){if(!p->used)break;if(p->buffer[0]!=COMM_SOF_0){drop(p,1);p->noise++;continue;}if(p->used<2)break;if(p->buffer[1]!=COMM_SOF_1){drop(p,1);p->noise++;continue;}if(p->used<6)break;if(p->buffer[5]>128){p->length_errors++;drop(p,1);continue;}n=(size_t)p->buffer[5]+8;if(p->used<n)break;s=frame_decode(p->buffer,n,&f);if(s==FRAME_OK){p->frames++;drop(p,n);cb(ctx,&f);}else{if(s==FRAME_CRC)p->crc_errors++;else if(s==FRAME_VERSION)p->version_errors++;drop(p,1);}}}return true;}
