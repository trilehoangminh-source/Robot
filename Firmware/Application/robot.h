#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H
#include "state_machine.h"
#include "safety.h"
#include "homing.h"
#include "gripper.h"
#include "joint_controller.h"
#include "trajectory_controller.h"
#include "command.h"
#include "telemetry.h"
/* All APIs single-owner. Communication submits objects through a future queue.
 * Callbacks never imply that output zero is mechanically safe. */
typedef struct {bool raw_active,debounced_active,valid;} robot_limit_t;
typedef struct {bool output_inhibited,physical_stop_confirmed;} robot_stop_result_t;
typedef struct {
 platform_status_t (*limits)(void *,uint8_t,robot_limit_t *);
 platform_status_t (*enable)(void *,bool);
 platform_status_t (*stop)(void *,robot_stop_result_t *);
 platform_status_t (*reference)(void *,uint8_t,float);
 void *context;
} robot_services_t;
typedef struct {
 bool simulation,stop_policy_verified,gripper_pulses_verified;
 trajectory_config_t trajectory;
 joint_config_t joint[3];safety_config_t safety;homing_config_t homing;
} robot_config_t;
typedef struct {
 platform_t platform,controller_platform;robot_services_t services;robot_config_t config;
 trajectory_controller_t trajectory;
 state_machine_t machine;safety_t safety;homing_t homing;gripper_t gripper;
 joint_controller_t joint[3];platform_encoder_feedback_t feedback[3];robot_limit_t limit[3];
 uint32_t step_id,now_ms,heartbeat_ms,busy_since;bool initialized,configured,heartbeat_valid,snapshot_valid;
 bool stop_pending,reference_ack,sync_ack,busy;robot_stop_result_t stop_result;
 uint16_t execution_faults;uint8_t event;command_t pending;bool command_pending;
} robot_t;
bool robot_initialize(robot_t *r,const platform_t *platform,const robot_services_t *services);
bool robot_configure(robot_t *r,const robot_config_t *config);
/* ACK code 0 proposed for Host agreement. Movement/gripper commands deferred to control step. */
bool robot_process_command(robot_t *r,const command_t *command,uint32_t now_ms,frame_t *response);
void robot_request_stop(robot_t *r);
bool robot_control_step(robot_t *r,uint32_t now_ms,const safety_snapshot_t *external);
void robot_application_step(robot_t *r,uint32_t now_ms);
uint8_t robot_get_state(const robot_t *r);
bool robot_get_telemetry(const robot_t *r,telemetry_snapshot_t *out);
bool robot_get_pending_event(robot_t *r,uint8_t *event);
#endif
