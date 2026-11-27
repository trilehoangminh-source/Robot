#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H
#include "protocol.h"
typedef struct {bool verified;float min_q[3],max_q[3],tracking_threshold[3];uint32_t tracking_ms,homing_ms;} safety_config_t;
typedef struct {
 bool feedback_valid,limit_valid,raw_limit,debounced_limit;
 bool limit_active,limit_direction_known,moving_into_limit,homing_approach;
 uint32_t raw_count;float measured_q,measured_w,estimated_q,estimated_w,tracking_error;
 bool controller_fault,driver_fault_valid,driver_fault;
} safety_joint_t;
typedef struct {
 uint32_t now_ms,heartbeat_ms,homing_start_ms;bool heartbeat_valid,operating,homing,homing_authorized,reference_valid;
 bool deadline_missed,trajectory_fault,estop_valid,estop_active;
 bool output_inhibited_valid,output_inhibited,physical_stop_valid,physical_stop;
 safety_joint_t joint[3];
} safety_snapshot_t;
typedef struct {
 safety_config_t config;bool configured,evaluated,tracking[3];uint32_t tracking_start[3];
 uint16_t active_faults,latched_faults;bool motion_permission,homing_permission,stop_requested;
 bool output_inhibited,physical_stop_confirmed;
} safety_t;
bool safety_initialize(safety_t *s,const safety_config_t *c);
bool safety_update(safety_t *s,const safety_snapshot_t *input);
/* Caller must remove causes, provide a fresh snapshot and explicitly authorize recovery. */
bool safety_reset_fault(safety_t *s,bool recovery_allowed);
#endif
