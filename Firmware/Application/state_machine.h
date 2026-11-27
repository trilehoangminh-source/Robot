#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H
#include "protocol.h"
typedef enum {SM_INIT_OK,SM_INIT_FAIL,SM_ENABLE,SM_DISABLE,SM_HOME,SM_HOME_DONE,SM_HOME_FAIL,SM_MOTION_START,SM_MOTION_DONE,SM_STOP,SM_FAULT_DETECTED,SM_RESET_FAULT,SM_REFERENCE_LOST} sm_event_t;
typedef struct {bool safe,homing_safe,reference_valid,recovery_allowed;uint16_t active_faults;} sm_guard_t;
typedef struct {uint8_t state;bool actuator_enable,homed,motion_permission,homing_permission,stop_requested,pending_motion;uint16_t latched_faults;} state_machine_t;
void state_machine_initialize(state_machine_t *s);
bool state_machine_process(state_machine_t *s,sm_event_t event,const sm_guard_t *guard);
bool state_machine_can_transition(const state_machine_t *s,sm_event_t event,const sm_guard_t *guard);
uint8_t state_machine_get_state(const state_machine_t *s);
bool state_machine_motion_allowed(const state_machine_t *s);
#endif
