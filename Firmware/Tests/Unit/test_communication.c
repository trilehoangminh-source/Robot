#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "crc.h"
#include "parser.h"
#include "command.h"
#include "telemetry.h"
static unsigned received;
static void accept(void *ctx,const frame_t *f){(void)ctx;assert(f->length<=128);received++;}
int main(void){
 frame_t f={0},decoded;parser_t p;command_t c;telemetry_snapshot_t t={0};
 uint8_t bytes[136],copy[136],noise;size_t n,i,split;unsigned rng=7;
 const uint8_t golden[]={170,85,1,32,7,12,0,0,128,63,0,0,0,192,0,0,0,63,173,93};
 assert(crc16_ccitt_false((const uint8_t *)"123456789",9)==0x29b1);
 f.id=COMM_CMD_SET_JOINT;f.seq=7;f.length=12;
 assert(protocol_put_float(f.payload,1)&&protocol_put_float(f.payload+4,-2)&&protocol_put_float(f.payload+8,0.5f));
 assert(frame_encode(&f,bytes,sizeof bytes,&n)==FRAME_OK&&n==sizeof golden&&memcmp(bytes,golden,n)==0);
 assert(frame_decode(bytes,n,&decoded)==FRAME_OK&&command_decode(&decoded,&c)==0&&c.q[1]==-2);
 for(split=0;split<=n;split++){assert(parser_initialize(&p,50));received=0;assert(parser_feed(&p,bytes,split,1,accept,0));assert(parser_feed(&p,bytes+split,n-split,2,accept,0));assert(received==1);}
 assert(parser_feed(&p,bytes,n,3,accept,0)&&received==2);
 memcpy(copy,bytes,n);copy[n-1]^=1;assert(parser_feed(&p,copy,n,4,accept,0));assert(p.crc_errors==1);
 assert(parser_feed(&p,bytes,n,5,accept,0)&&received==3);
 assert(parser_feed(&p,bytes,7,6,accept,0));parser_expire(&p,60);assert(p.timeouts==1);
 assert(parser_feed(&p,bytes,n,61,accept,0)&&received==4);
 memcpy(copy,bytes,n);copy[2]=2;assert(frame_decode(copy,n,&decoded)==FRAME_VERSION);
 copy[5]=129;assert(parser_feed(&p,copy,n,62,accept,0)&&p.length_errors>0);
 assert(frame_encode(&f,bytes,1,&n)==FRAME_LENGTH);
 f.length=128;memset(f.payload,0xaa,128);f.payload[5]=0x55;
 assert(frame_encode(&f,bytes,sizeof bytes,&n)==FRAME_OK&&n==136);
 assert(parser_initialize(&p,50));received=0;assert(parser_feed(&p,bytes,n,0,accept,0)&&received==1);
 f.length=0;f.id=COMM_CMD_PING;assert(frame_encode(&f,bytes,sizeof bytes,&n)==FRAME_OK&&n==8);
 assert(command_decode(&f,&c)==0);
 {const uint8_t ids[]={1,2,3,0x10,0x11,0x12,0x24,0x40,0x41,0x50};for(i=0;i<sizeof ids;i++){f.id=ids[i];assert(command_decode(&f,&c)==0);}}
 f.id=COMM_CMD_GRIPPER;f.length=1;f.payload[0]=2;assert(command_decode(&f,&c)==NACK_PARAMETER);f.payload[0]=1;assert(command_decode(&f,&c)==0);
 f.id=COMM_CMD_TRAJECTORY_BEGIN;f.length=4;protocol_put_u16(f.payload,4);protocol_put_u16(f.payload+2,128);assert(command_decode(&f,&c)==0);
 f.id=COMM_CMD_TRAJECTORY_START;f.length=2;assert(command_decode(&f,&c)==0);
 f.id=COMM_CMD_TRAJECTORY_POINT;f.length=20;memset(f.payload,0,20);assert(command_decode(&f,&c)==0);
 f.id=COMM_CMD_SET_JOINT;f.length=12;protocol_put_u32(f.payload,0x7fc00000U);assert(command_decode(&f,&c)==NACK_PARAMETER);protocol_put_u32(f.payload,0x7f800000U);assert(command_decode(&f,&c)==NACK_PARAMETER);
 t.feedback_valid=true;t.gripper=2;t.time_ms=0x12345678;t.code=1;t.q[0]=1;
 {const uint8_t ids[]={0x80,0x81,0x82,0x83,0x84,0x90,0x91,0xa0,0xa1};const uint8_t lens[]={2,2,0,8,34,34,40,6,5};for(i=0;i<sizeof ids;i++){assert(telemetry_encode(ids[i],3,&t,&decoded)&&decoded.length==lens[i]);}}
 assert(telemetry_encode(COMM_MSG_JOINT_STATE,0,&t,&decoded));assert(decoded.payload[0]==0x78&&decoded.payload[7]==0x3f&&decoded.payload[33]==2);
 t.feedback_valid=false;assert(!telemetry_encode(COMM_MSG_JOINT_STATE,0,&t,&decoded));
 assert(parser_initialize(&p,50));for(i=0;i<100000;i++){rng=rng*1664525U+1013904223U;noise=(uint8_t)(rng>>24);assert(parser_feed(&p,&noise,1,(uint32_t)i,accept,0));assert(p.used<=136);}
 parser_expire(&p,100100);assert(parser_feed(&p,bytes,n,100101,accept,0));
 puts("communication: PASS; Python struct/crc golden vector matched");return 0;
}
