#ifndef ROBOT_ROBOT_TASK_H
#define ROBOT_ROBOT_TASK_H
#include "shared_data.h"
#define ROBOT_TASK_PERIOD_MS 10U
typedef struct {uint32_t next_ms,releases,missed_periods,pending_requests,startup_blocked;} robot_task_t;
void robot_task_initialize(robot_task_t *task,uint32_t now);
bool robot_task_step(robot_task_t *task,uint32_t now);
void robot_task_run(void *argument);
#endif
