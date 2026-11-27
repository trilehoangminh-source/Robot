#include "robot_task.h"
#include <string.h>
void robot_task_initialize(robot_task_t *t,uint32_t now){if(!t)return;memset(t,0,sizeof *t);t->next_ms=now+ROBOT_TASK_PERIOD_MS;}
bool robot_task_step(robot_task_t *t,uint32_t now){uint32_t late,skipped;shared_snapshot_t snapshot;if(!t)return false;late=now-t->next_ms;if(late>=0x80000000U)return false;skipped=late/ROBOT_TASK_PERIOD_MS;t->missed_periods+=skipped;t->next_ms+=(skipped+1U)*ROBOT_TASK_PERIOD_MS;t->releases++;if(!shared_snapshot_read(&snapshot)||snapshot.startup_blocked){t->startup_blocked++;return true;}if(!shared_application_request(now))t->pending_requests++;return true;}
#ifndef TASKS_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_timebase.h"
void robot_task_run(void *argument){static robot_task_t state;TickType_t anchor=xTaskGetTickCount(),period=pdMS_TO_TICKS(ROBOT_TASK_PERIOD_MS);(void)argument;robot_task_initialize(&state,bsp_timebase_now_ms());
 for(;;){TickType_t now,late;vTaskDelayUntil(&anchor,period);(void)robot_task_step(&state,bsp_timebase_now_ms());now=xTaskGetTickCount();late=now-anchor;if(late>=period)anchor+=(late/period)*period;}
}
#endif
