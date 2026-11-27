#ifndef ROBOT_TELEMETRY_TASK_H
#define ROBOT_TELEMETRY_TASK_H
#include "shared_data.h"
typedef struct {uint32_t next_ms,generated,enqueued,dropped,invalid,stale,startup_blocked,missed_periods;} telemetry_task_t;
void telemetry_task_initialize(telemetry_task_t *task,uint32_t now);
void telemetry_task_step(telemetry_task_t *task,uint32_t now,bool debug_enabled);
void telemetry_task_run(void *argument);
#endif
