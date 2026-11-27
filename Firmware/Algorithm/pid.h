#ifndef ROBOT_PID_H
#define ROBOT_PID_H
#include <stdbool.h>
typedef struct {float kp,ki,kd,dt,out_min,out_max,i_min,i_max;} pid_config_t;
typedef struct {pid_config_t config; float integral; bool configured;} robot_pid_t;
bool pid_initialize(robot_pid_t *p,const pid_config_t *c);
bool pid_configure(robot_pid_t *p,const pid_config_t *c);
bool pid_reset(robot_pid_t *p);
/* velocity is measured/estimated velocity, derivative term = -Kd * velocity. */
bool pid_update(robot_pid_t *p,float reference,float position,float velocity,float *output);
#endif
