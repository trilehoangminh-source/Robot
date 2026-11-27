#ifndef ROBOT_HOMING_H
#define ROBOT_HOMING_H
#include "platform.h"
typedef enum {HOME_IDLE,HOME_CHECK,HOME_RELEASE,HOME_SEEK,HOME_BACKOFF,HOME_SLOW,HOME_REFERENCE,HOME_DONE,HOME_FAULT,HOME_ABORTED} homing_phase_t;
typedef struct {int direction;float seek,slow,backoff,backoff_rad,home_angle,max_travel;uint32_t phase_timeout_ms;} homing_joint_config_t;
typedef struct {bool verified;uint8_t order[3];uint32_t total_timeout_ms;homing_joint_config_t joint[3];} homing_config_t;
typedef struct {uint32_t now_ms;bool permission,safe,feedback_valid,raw_active,debounced_active,switch_valid,driver_fault,stop_confirmed,reference_committed,controller_synchronized;float q;} homing_input_t;
typedef struct {uint8_t joint;float output,home_angle;bool motion,stop_requested,reference_request;} homing_intent_t;
typedef struct {homing_config_t config;homing_phase_t phase;uint8_t index;bool configured,homed[3];uint32_t start_ms,phase_ms;float start_q,backoff_q;homing_intent_t intent;} homing_t;
bool homing_initialize(homing_t *h,const homing_config_t *config);
bool homing_start(homing_t *h,const homing_input_t *in);
void homing_abort(homing_t *h);
bool homing_update(homing_t *h,const homing_input_t *in);
#endif
