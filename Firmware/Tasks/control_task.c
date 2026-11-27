#include "control_task.h"
bool control_task_deadline(uint32_t n,uint32_t elapsed,uint32_t period){return !period||n!=1U||elapsed>=period;}
#ifndef TASKS_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_timer.h"
#include "bsp_timebase.h"
#include "main.h"
static robot_t *owner;
static control_inputs_t inputs;
static void *input_context;
static TaskHandle_t control_handle;
static volatile uint32_t release_cycle;
static bool task_started;
static uint16_t reported_faults;
bool control_task_bind(robot_t *r,control_inputs_t provider,void *ctx){if(task_started||!r||!r->configured||!provider)return false;owner=r;inputs=provider;input_context=ctx;return true;}
static void timer_release(void *ctx){BaseType_t wake=pdFALSE;(void)ctx;
 if(!control_handle||xTaskGetSchedulerState()!=taskSCHEDULER_RUNNING)return;
 release_cycle=DWT->CYCCNT;vTaskNotifyGiveFromISR(control_handle,&wake);portYIELD_FROM_ISR(wake);
}
void control_task_run(void *argument){static shared_snapshot_t snapshot;uint32_t period,last=0;bool first=true;(void)argument;task_started=true;
 snapshot.startup_blocked=true;shared_snapshot_publish(&snapshot);
 if(!owner||!inputs){vTaskSuspend(NULL);for(;;)vTaskSuspend(NULL);}
 control_handle=xTaskGetCurrentTaskHandle();
 if(xTaskGetSchedulerState()!=taskSCHEDULER_RUNNING||NVIC_GetPriority(TIM6_DAC_IRQn)!=5U||NVIC_GetPriority(TIM6_DAC_IRQn)<configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY){vTaskSuspend(NULL);}
 CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
 snapshot.timing_valid=(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)!=0U;
 period=SystemCoreClock/1000U;
 (void)ulTaskNotifyTake(pdTRUE,0);NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
 if(!snapshot.timing_valid||bsp_timer_start_control((osThreadId_t)control_handle,timer_release,0)!=BSP_OK){vTaskSuspend(NULL);}
 snapshot.startup_blocked=false;
 for(;;){uint32_t n=ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(2));uint32_t begin=DWT->CYCCNT,now=bsp_timebase_now_ms(),heartbeat;
 bool missed=n!=1U||(!first&&(uint32_t)(begin-last)>period+period/2U);safety_snapshot_t external={0};command_t command;frame_t response;unsigned i;uint8_t event;
 snapshot.wake_latency_cycles=begin-release_cycle;if(snapshot.wake_latency_cycles>snapshot.max_wake_latency_cycles)snapshot.max_wake_latency_cycles=snapshot.wake_latency_cycles;
 first=false;last=begin;if(n>1U)snapshot.notification_overruns+=n-1U;
 if(shared_stop_command_take(&command)){if(robot_process_command(owner,&command,now,&response))shared_stop_response(&response);}else if(shared_stop_take())robot_request_stop(owner);
 if(shared_heartbeat_take(&heartbeat)){command_t hb={0};hb.id=COMM_CMD_HEARTBEAT;(void)robot_process_command(owner,&hb,heartbeat,&response);}
 for(i=0;i<2;i++){if(shared_stop_take()){robot_request_stop(owner);break;}if(!shared_response_available()||!shared_command_pop(&command))break;
 if(robot_process_command(owner,&command,now,&response)&&!shared_response_push(&response)){robot_request_stop(owner);break;}}
 if(!inputs(input_context,now,&external)){missed=true;robot_request_stop(owner);}
 external.deadline_missed|=missed;
 (void)robot_control_step(owner,now,&external);
 {uint32_t requested;if(shared_application_take(&requested)){uint32_t app_begin=DWT->CYCCNT;(void)requested;robot_application_step(owner,now);snapshot.application_cycles=DWT->CYCCNT-app_begin;if(snapshot.application_cycles>snapshot.max_application_cycles)snapshot.max_application_cycles=snapshot.application_cycles;snapshot.application_steps++;}}
 snapshot.execution_cycles=DWT->CYCCNT-begin;
 if(snapshot.execution_cycles>snapshot.max_execution_cycles)snapshot.max_execution_cycles=snapshot.execution_cycles;
 if(control_task_deadline(n,snapshot.execution_cycles,period)||missed){snapshot.deadline_misses++;robot_request_stop(owner);
 /* Explicit latched Safety deadline; no extra encoder/PID step. */
 owner->safety.active_faults|=FAULT_DEADLINE;owner->safety.latched_faults|=FAULT_DEADLINE;}
 snapshot.steps++;snapshot.state=robot_get_state(owner);snapshot.active_faults=owner->safety.active_faults;snapshot.latched_faults=owner->safety.latched_faults;
 snapshot.published_ms=now;snapshot.telemetry_valid=robot_get_telemetry(owner,&snapshot.telemetry);snapshot.debug_valid=snapshot.telemetry_valid;for(i=0;i<3;i++){snapshot.joint[i]=owner->joint[i].diagnostics;if(!owner->joint[i].reference_valid||!owner->joint[i].feedback.valid)snapshot.debug_valid=false;}
 if(robot_get_pending_event(owner,&event)){telemetry_snapshot_t t={0};t.code=event;t.trajectory_id=owner->trajectory.id;if(telemetry_encode(COMM_MSG_EVENT,0,&t,&response)&&!shared_response_push(&response))robot_request_stop(owner);}
 if(snapshot.latched_faults!=reported_faults){telemetry_snapshot_t t={0};t.time_ms=now;t.fault_flags=snapshot.latched_faults;if(telemetry_encode(COMM_MSG_FAULT,0,&t,&response)&&shared_response_push(&response))reported_faults=snapshot.latched_faults;}
 shared_snapshot_publish(&snapshot);
 }
}
#endif
