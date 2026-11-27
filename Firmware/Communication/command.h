#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H
#include "frame.h"
typedef struct {uint8_t id,seq,gripper;uint16_t trajectory_id,index,point_count;uint32_t time_ms;float q[3];} command_t;
/* Returns 0 for structurally valid command; execution permission belongs to Application. */
uint8_t command_decode(const frame_t *frame,command_t *out);
#endif
