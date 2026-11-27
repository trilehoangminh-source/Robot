#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "communication_config.h"
enum {PROTO_INIT,PROTO_IDLE,PROTO_HOMING,PROTO_READY,PROTO_RUNNING,PROTO_STOPPED,PROTO_FAULT};
enum {NACK_UNSUPPORTED=1,NACK_LENGTH,NACK_PARAMETER,NACK_STATE,NACK_NOT_HOMED,NACK_LIMIT,NACK_BUFFER_FULL,NACK_TRAJECTORY,NACK_BUSY,NACK_FAULT,NACK_NOT_IMPLEMENTED,NACK_SEQUENCE};
enum {EVENT_HOMING_DONE=1,EVENT_MOTION_DONE,EVENT_TRAJECTORY_DONE,EVENT_STOP_DONE};
enum {FAULT_COMM=1,FAULT_POSITION=2,FAULT_SWITCH=4,FAULT_ENCODER=8,FAULT_TRACKING=16,FAULT_HOMING=32,FAULT_MOTOR=64,FAULT_DEADLINE=128,FAULT_TRAJECTORY=256,FAULT_ESTOP=512};
void protocol_put_u16(uint8_t *p,uint16_t v);
void protocol_put_u32(uint8_t *p,uint32_t v);
uint16_t protocol_u16(const uint8_t *p);
uint32_t protocol_u32(const uint8_t *p);
bool protocol_put_float(uint8_t *p,float v);
bool protocol_float(const uint8_t *p,float *v);
#endif
