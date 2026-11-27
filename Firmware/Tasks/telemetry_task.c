#include "telemetry_task.h"
#include <string.h>
void telemetry_task_initialize(telemetry_task_t *t,uint32_t now){if(!t)return;memset(t,0,sizeof *t);t->next_ms=now+COMM_TELEMETRY_PERIOD_MS;}
static void emit(telemetry_task_t *t,uint8_t id,const telemetry_snapshot_t *s){frame_t frame;if(!telemetry_encode(id,0,s,&frame)){t->invalid++;return;}t->generated++;if(shared_telemetry_push(&frame))t->enqueued++;else t->dropped++;}
void telemetry_task_step(telemetry_task_t *t,uint32_t now,bool debug){shared_snapshot_t snapshot;uint32_t late,skipped;if(!t)return;late=now-t->next_ms;if(late>=0x80000000U)return;skipped=late/COMM_TELEMETRY_PERIOD_MS;t->missed_periods+=skipped;t->next_ms+=(skipped+1U)*COMM_TELEMETRY_PERIOD_MS;
 if(!shared_snapshot_read(&snapshot)||snapshot.startup_blocked){t->startup_blocked++;return;}
 if((uint32_t)(now-snapshot.published_ms)>COMM_TELEMETRY_MAX_AGE_MS||(uint32_t)(now-snapshot.telemetry.time_ms)>COMM_TELEMETRY_MAX_AGE_MS){t->stale++;return;}
 if(!snapshot.telemetry_valid||!snapshot.telemetry.feedback_valid){t->invalid++;return;}
 emit(t,COMM_MSG_JOINT_STATE,&snapshot.telemetry);
 if(debug){if(snapshot.debug_valid)emit(t,COMM_MSG_CONTROL_DEBUG,&snapshot.telemetry);else t->invalid++;}
}
#ifndef TASKS_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_timebase.h"
void telemetry_task_run(void *argument){static telemetry_task_t state;TickType_t anchor=xTaskGetTickCount(),period=pdMS_TO_TICKS(COMM_TELEMETRY_PERIOD_MS);(void)argument;telemetry_task_initialize(&state,bsp_timebase_now_ms());
 for(;;){TickType_t now,late;vTaskDelayUntil(&anchor,period);telemetry_task_step(&state,bsp_timebase_now_ms(),COMM_CONTROL_DEBUG_ENABLED!=0U);now=xTaskGetTickCount();late=now-anchor;if(late>=period)anchor+=(late/period)*period;}
}
#endif
