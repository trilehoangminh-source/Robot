#ifndef ROBOT_PLATFORM_H
#define ROBOT_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PLATFORM_OK = 0,
    PLATFORM_INVALID_ARGUMENT,
    PLATFORM_NOT_READY,
    PLATFORM_NOT_IMPLEMENTED,
    PLATFORM_BUSY,
    PLATFORM_IO_ERROR
} platform_status_t;

typedef enum {
    PLATFORM_GRIPPER_OPEN = 0,
    PLATFORM_GRIPPER_CLOSE = 1,
    PLATFORM_GRIPPER_UNKNOWN = 2
} platform_gripper_command_t;

typedef struct {
 uint32_t raw,reference_epoch; int64_t count;
 float q,w; bool valid;
} platform_encoder_feedback_t;

typedef struct platform_ops {
    /* Raw hardware counter, modulo 2^32 (J1) or 2^16 (J2/J3).
     * Conversion to radians requires validated CPR, sign and home offset. */
    platform_status_t (*read_encoder)(void *context, uint8_t joint, uint32_t *raw_count);
    /* 'active' is the logical limit state, not the electrical pin level. */
    platform_status_t (*read_limit)(void *context, uint8_t joint, bool *active);
    platform_status_t (*time_ms)(void *context, uint32_t *now_ms);
    /* Signed normalized command in [-1, +1]; caller owns safety and limits. */
    platform_status_t (*set_motor_pwm)(void *context, uint8_t joint, float command);
    platform_status_t (*set_gripper)(void *context, platform_gripper_command_t command);
    platform_status_t (*encoder_feedback)(void *context,uint8_t joint,uint32_t step_id,uint32_t dt_ms,platform_encoder_feedback_t *out);
    platform_status_t (*cancel_motor)(void *context,uint8_t joint);
} platform_ops_t;

typedef struct {
    const platform_ops_t *ops;
    void *context;
} platform_t;

static inline platform_status_t platform_read_encoder(const platform_t *platform,
                                                       uint8_t joint, uint32_t *raw_count)
{
    if (platform == 0 || raw_count == 0 || joint >= ROBOT_JOINT_COUNT)
        return PLATFORM_INVALID_ARGUMENT;
    if (platform->ops == 0 || platform->ops->read_encoder == 0)
        return PLATFORM_NOT_READY;
    return platform->ops->read_encoder(platform->context, joint, raw_count);
}

static inline platform_status_t platform_read_limit(const platform_t *platform,
                                                     uint8_t joint, bool *active)
{
    if (platform == 0 || active == 0 || joint >= ROBOT_JOINT_COUNT)
        return PLATFORM_INVALID_ARGUMENT;
    if (platform->ops == 0 || platform->ops->read_limit == 0)
        return PLATFORM_NOT_READY;
    return platform->ops->read_limit(platform->context, joint, active);
}

static inline platform_status_t platform_time_ms(const platform_t *platform,
                                                  uint32_t *now_ms)
{
    if (platform == 0 || now_ms == 0)
        return PLATFORM_INVALID_ARGUMENT;
    if (platform->ops == 0 || platform->ops->time_ms == 0)
        return PLATFORM_NOT_READY;
    return platform->ops->time_ms(platform->context, now_ms);
}

static inline platform_status_t platform_set_motor_pwm(const platform_t *platform,
                                                        uint8_t joint, float command)
{
    if (platform == 0 || joint >= ROBOT_JOINT_COUNT ||
        !(command >= -1.0f && command <= 1.0f))
        return PLATFORM_INVALID_ARGUMENT;
    if (platform->ops == 0 || platform->ops->set_motor_pwm == 0)
        return PLATFORM_NOT_READY;
    return platform->ops->set_motor_pwm(platform->context, joint, command);
}

static inline platform_status_t platform_set_gripper(const platform_t *platform,
                                                     platform_gripper_command_t command)
{
    if (platform == 0 || (command != PLATFORM_GRIPPER_OPEN &&
                          command != PLATFORM_GRIPPER_CLOSE))
        return PLATFORM_INVALID_ARGUMENT;
    if (platform->ops == 0 || platform->ops->set_gripper == 0)
        return PLATFORM_NOT_READY;
    return platform->ops->set_gripper(platform->context, command);
}

/* STM32 binding uses HAL only inside platform_stm32.c. Call after HAL init.
 * Binding alone does not start timers, PWM, DMA or motors. */
platform_status_t platform_stm32_bind(platform_t *platform);

/* Simulation bridge supplies observations and consumes commands. Setting ready
 * means the bridge is attached; it does not imply calibrated robot physics. */
typedef struct {
    platform_encoder_feedback_t feedback[ROBOT_JOINT_COUNT];
    bool ready;
    uint32_t encoder_raw[ROBOT_JOINT_COUNT];
    bool limit_active[ROBOT_JOINT_COUNT];
    uint32_t now_ms;
    float motor_command[ROBOT_JOINT_COUNT];
    platform_gripper_command_t gripper_command;
} platform_sim_state_t;

platform_status_t platform_sim_bind(platform_t *platform, platform_sim_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
