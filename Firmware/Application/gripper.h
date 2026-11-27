#ifndef ROBOT_GRIPPER_H
#define ROBOT_GRIPPER_H
#include "platform.h"
typedef struct {platform_t platform;bool configured,accepted,output_known,mechanical_known,disable_requested;uint8_t command;} gripper_t;
bool gripper_initialize(gripper_t *g,const platform_t *p);
bool gripper_configure(gripper_t *g,bool pulses_verified);
platform_status_t gripper_request(gripper_t *g,bool close,bool permission);
void gripper_abort(gripper_t *g);
const gripper_t *gripper_get_status(const gripper_t *g);
#endif
