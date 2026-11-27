#ifndef ROBOT_CONTROL_TASK_H
#define ROBOT_CONTROL_TASK_H
#include "shared_data.h"
/* Call only before task starts. Provider supplies real Safety inputs, not fabricated validity.
 * No binding is installed by generated startup; default task remains blocked. */
typedef bool (*control_inputs_t)(void *context,uint32_t now,safety_snapshot_t *out);
bool control_task_bind(robot_t *robot,control_inputs_t provider,void *context);
void control_task_run(void *argument);
/* Portable timing classifier: a missed release triggers one fault step, never catch-up PID. */
bool control_task_deadline(uint32_t notifications,uint32_t elapsed_cycles,uint32_t period_cycles);
#endif
