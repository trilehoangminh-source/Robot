#include "telemetry.h"
bool telemetry_encode(uint8_t id,uint8_t seq,const telemetry_snapshot_t *s,frame_t *out){
 frame_t f={0};unsigned i;f.id=id;f.seq=seq;if(!out)return false;
 if(id==COMM_MSG_PONG){*out=f;return true;}if(!s)return false;
 switch(id){
 case COMM_MSG_ACK:case COMM_MSG_NACK:
 if(id==COMM_MSG_NACK&&(s->code<1||s->code>12))return false;
 f.length=2;f.payload[0]=s->request_id;f.payload[1]=s->code;break;
 case COMM_MSG_ROBOT_INFO:
 f.length=8;f.payload[0]=s->fw_major;f.payload[1]=s->fw_minor;f.payload[2]=s->fw_patch;f.payload[3]=3;
 protocol_put_u16(f.payload+4,COMM_MAX_PAYLOAD_BYTES);protocol_put_u16(f.payload+6,COMM_TRAJECTORY_CAPACITY_POINTS);break;
 case COMM_MSG_STATE_REPLY:case COMM_MSG_JOINT_STATE:
 if(!s->feedback_valid||s->state>PROTO_FAULT||s->limit_flags>7||s->homing_flags>7||s->gripper>2)return false;
 f.length=34;protocol_put_u32(f.payload,s->time_ms);
 for(i=0;i<3;i++)if(!protocol_put_float(f.payload+4+4*i,s->q[i])||!protocol_put_float(f.payload+16+4*i,s->w[i]))return false;
 f.payload[28]=s->state;f.payload[29]=s->limit_flags;f.payload[30]=s->homing_flags;
 protocol_put_u16(f.payload+31,s->fault_flags);f.payload[33]=s->gripper;break;
 case COMM_MSG_CONTROL_DEBUG:
 if(!s->feedback_valid)return false;f.length=40;protocol_put_u32(f.payload,s->time_ms);
 for(i=0;i<3;i++)if(!protocol_put_float(f.payload+4+4*i,s->reference[i])||!protocol_put_float(f.payload+16+4*i,s->error[i])||!protocol_put_float(f.payload+28+4*i,s->output[i]))return false;break;
 case COMM_MSG_FAULT:f.length=6;protocol_put_u32(f.payload,s->time_ms);protocol_put_u16(f.payload+4,s->fault_flags);break;
 case COMM_MSG_EVENT:if(s->code<1||s->code>4)return false;f.length=5;f.payload[0]=s->code;protocol_put_u16(f.payload+1,s->trajectory_id);protocol_put_u16(f.payload+3,s->detail);break;
 default:return false;
 }*out=f;return true;
}
