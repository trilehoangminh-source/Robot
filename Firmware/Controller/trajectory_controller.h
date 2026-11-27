#ifndef ROBOT_TRAJECTORY_CONTROLLER_H
#define ROBOT_TRAJECTORY_CONTROLLER_H
#include "communication_config.h"
#include <stdbool.h>
#include <stdint.h>
typedef enum {TRAJ_EMPTY,TRAJ_LOADING,TRAJ_RUNNING,TRAJ_SETTLING,TRAJ_DONE,TRAJ_ABORTED} trajectory_state_t;
typedef struct {bool verified;float min_q[3],max_q[3],max_velocity[3],max_acceleration[3],start_tolerance[3],complete_error[3],complete_velocity[3];uint32_t settle_ms;} trajectory_config_t;
typedef struct {uint32_t time_ms;float q[3];} trajectory_point_t;
typedef struct {trajectory_config_t config;trajectory_point_t points[COMM_TRAJECTORY_CAPACITY_POINTS];uint16_t id,expected,received,segment;trajectory_state_t state;uint32_t start_ms,stable_ms,last_ms;bool configured,stable,clock_valid;} trajectory_controller_t;
void trajectory_initialize(trajectory_controller_t *t);
bool trajectory_configure(trajectory_controller_t *t,const trajectory_config_t *c);
bool trajectory_begin(trajectory_controller_t *t,uint16_t id,uint16_t count);
bool trajectory_append(trajectory_controller_t *t,uint16_t id,uint16_t index,uint32_t time_ms,const float q[3]);
/* Validation checks velocity and sampled acceleration jumps over 1 ms, including
 * start/end from rest. It cannot prove bounded continuous acceleration. */
bool trajectory_validate(const trajectory_controller_t *t,const float current[3]);
bool trajectory_start(trajectory_controller_t *t,uint16_t id,uint32_t now_ms,const float current[3]);
bool trajectory_update(trajectory_controller_t *t,uint32_t now_ms,const float measured[3],const float velocity[3],float reference[3]);
void trajectory_abort(trajectory_controller_t *t);
bool trajectory_clear(trajectory_controller_t *t);
trajectory_state_t trajectory_get_status(const trajectory_controller_t *t);
#endif
