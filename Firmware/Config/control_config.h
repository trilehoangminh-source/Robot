#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include <stdint.h>

#define CONTROL_PERIOD_MS 1U
#define ROBOT_TASK_PERIOD_MS 10U
#define TELEMETRY_TASK_PERIOD_MS 20U
#define CONTROL_PWM_MIN (-1.0f)
#define CONTROL_PWM_MAX (1.0f)

/* No PID gains, actuator polarity, stop policy or safe gripper pulses are verified.
 * A controller must refuse motion until these are supplied and checked. */
#define CONTROL_PID_GAINS_VERIFIED 0U
#define CONTROL_MOTOR_POLARITY_VERIFIED 0U
#define CONTROL_STOP_POLICY_VERIFIED 0U
#define CONTROL_GRIPPER_PULSES_VERIFIED 0U

typedef struct {
    float kp;
    float ki;
    float kd;
    uint8_t valid;
} control_pid_gains_t;

#endif
