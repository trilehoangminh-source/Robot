#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H
#include "frame.h"
typedef struct {
 bool feedback_valid;uint32_t time_ms;float q[3],w[3],reference[3],error[3],output[3];
 uint8_t state,limit_flags,homing_flags,gripper;
 uint16_t fault_flags,trajectory_id,detail;
 uint8_t request_id,code,fw_major,fw_minor,fw_patch;
} telemetry_snapshot_t;
/* code is caller-supplied ACK status or agreed NACK/Event code. No execution. */
bool telemetry_encode(uint8_t id,uint8_t seq,const telemetry_snapshot_t *s,frame_t *out);
#endif
