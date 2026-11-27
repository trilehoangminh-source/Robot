#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H
#include "pid.h"
#include "kalman.h"
#include "platform.h"
typedef enum { JOINT_OK=0, JOINT_INVALID, JOINT_NOT_READY, JOINT_FAULT } joint_status_t;
typedef struct {
 robot_joint_calibration_t calibration; pid_config_t pid; kalman_config_t kalman;
 /* Explicit raw counter at the calibrated reference; reset/homing requires reconfigure. */
 uint32_t reference_raw; bool parameters_verified; uint32_t busy_timeout_ms;
} joint_config_t;
typedef struct {
 uint32_t raw; int64_t counts; float measured_q,measured_w,estimated_q,estimated_w;
 bool valid;
} joint_feedback_t;
typedef struct {
 float reference,error,requested_output; bool write_accepted,applied_output_known;
 platform_status_t platform_status;
} joint_diagnostics_t;
typedef struct {
 uint8_t joint; platform_t platform; joint_config_t config; robot_pid_t pid; kalman_t kalman;
 joint_feedback_t feedback; joint_diagnostics_t diagnostics;
 uint32_t step_id,epoch,busy_steps;
 bool initialized,configured,enabled,fault,reference_valid,sampled;
} joint_controller_t;
joint_status_t joint_controller_initialize(joint_controller_t *j,uint8_t id,const platform_t *p);
joint_status_t joint_controller_configure(joint_controller_t *j,const joint_config_t *c);
/* Explicit fault acknowledgement; invalidates calibration/reference after homing.
 * Reconfigure and obtain new feedback/reference before re-enabling. No output write. */
joint_status_t joint_controller_reset(joint_controller_t *j);
joint_status_t joint_controller_enable(joint_controller_t *j);
/* Revokes logical ownership only; caller handles robot-wide physical stop. */
joint_status_t joint_controller_disable(joint_controller_t *j);
joint_status_t joint_controller_set_reference(joint_controller_t *j,float q);
joint_status_t joint_controller_update(joint_controller_t *j);
joint_status_t joint_controller_get_feedback(const joint_controller_t *j,joint_feedback_t *out);
joint_status_t joint_controller_get_diagnostics(const joint_controller_t *j,joint_diagnostics_t *out);
joint_status_t joint_controller_get_status(const joint_controller_t *j);
#endif
