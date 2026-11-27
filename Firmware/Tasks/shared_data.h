#ifndef ROBOT_SHARED_DATA_H
#define ROBOT_SHARED_DATA_H
#include "robot.h"
#define SHARED_COMMAND_CAPACITY 8U
#define SHARED_RESPONSE_CAPACITY 8U
typedef struct {
 uint32_t steps,notification_overruns,deadline_misses,execution_cycles,max_execution_cycles,wake_latency_cycles,max_wake_latency_cycles;
 uint32_t published_ms,application_steps,application_cycles,max_application_cycles;
 bool debug_valid;
 bool timing_valid,startup_blocked;telemetry_snapshot_t telemetry;bool telemetry_valid;
 joint_diagnostics_t joint[3];uint8_t state;
 uint16_t active_faults,latched_faults;
} shared_snapshot_t;
/* Task-context only, initialized before scheduler. Static bounded rings;
 * multi-task access uses short FreeRTOS critical sections, no blocking. */
void shared_data_initialize(void);
bool shared_command_push(const command_t *command);
bool shared_command_pop(command_t *command);
void shared_stop_request(void);
void shared_stop_command(uint8_t seq);
bool shared_stop_command_take(command_t *command);
void shared_stop_response(const frame_t *response);
bool shared_stop_response_take(frame_t *response);
bool shared_response_available(void);
bool shared_stop_take(void);
void shared_heartbeat_publish(uint32_t time_ms);
bool shared_heartbeat_take(uint32_t *time_ms);
bool shared_response_push(const frame_t *response);
bool shared_response_pop(frame_t *response);
uint32_t shared_response_dropped(void);
void shared_snapshot_publish(const shared_snapshot_t *snapshot);
bool shared_snapshot_read(shared_snapshot_t *snapshot);
typedef struct {uint32_t requested,taken,pending_collisions,last_request_ms;bool pending;} shared_application_diagnostics_t;
typedef struct {uint32_t enqueued,replaced,completed,transport_dropped;} shared_telemetry_diagnostics_t;
bool shared_application_request(uint32_t now);
bool shared_application_take(uint32_t *requested_ms);
void shared_application_diagnostics(shared_application_diagnostics_t *out);
bool shared_telemetry_push(const frame_t *frame);
bool shared_telemetry_pop(frame_t *frame);
void shared_telemetry_complete(void);
void shared_telemetry_transport_drop(void);
void shared_telemetry_diagnostics(shared_telemetry_diagnostics_t *out);
#endif
