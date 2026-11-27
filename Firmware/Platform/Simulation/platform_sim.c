#include "platform.h"

static platform_status_t sim_read_encoder(void *context, uint8_t joint,
                                          uint32_t *raw_count)
{
    platform_sim_state_t *state = (platform_sim_state_t *)context;
    if (!state->ready)
        return PLATFORM_NOT_READY;
    *raw_count = state->encoder_raw[joint];
    return PLATFORM_OK;
}

static platform_status_t sim_read_limit(void *context, uint8_t joint, bool *active)
{
    platform_sim_state_t *state = (platform_sim_state_t *)context;
    if (!state->ready)
        return PLATFORM_NOT_READY;
    *active = state->limit_active[joint];
    return PLATFORM_OK;
}

static platform_status_t sim_time_ms(void *context, uint32_t *now_ms)
{
    platform_sim_state_t *state = (platform_sim_state_t *)context;
    if (!state->ready)
        return PLATFORM_NOT_READY;
    *now_ms = state->now_ms;
    return PLATFORM_OK;
}

static platform_status_t sim_set_motor_pwm(void *context, uint8_t joint,
                                            float command)
{
    platform_sim_state_t *state = (platform_sim_state_t *)context;
    if (!state->ready)
        return PLATFORM_NOT_READY;
    state->motor_command[joint] = command;
    return PLATFORM_OK; /* Command recorded for the simulation bridge to consume. */
}

static platform_status_t sim_set_gripper(void *context,
                                         platform_gripper_command_t command)
{
    platform_sim_state_t *state = (platform_sim_state_t *)context;
    if (!state->ready)
        return PLATFORM_NOT_READY;
    state->gripper_command = command;
    return PLATFORM_OK;
}

static platform_status_t sim_feedback(void *ctx,uint8_t j,uint32_t step,uint32_t dt,platform_encoder_feedback_t *out){
 platform_sim_state_t *s=(platform_sim_state_t *)ctx;(void)step;(void)dt;
 if(!s->ready)return PLATFORM_NOT_READY;*out=s->feedback[j];return out->valid?PLATFORM_OK:PLATFORM_NOT_READY;
}
static platform_status_t sim_cancel(void *ctx,uint8_t j){platform_sim_state_t *s=(platform_sim_state_t *)ctx;s->motor_command[j]=0;return PLATFORM_OK;}
static const platform_ops_t sim_ops = {
    sim_read_encoder,
    sim_read_limit,
    sim_time_ms,
    sim_set_motor_pwm,
    sim_set_gripper,
    sim_feedback,
    sim_cancel
};

platform_status_t platform_sim_bind(platform_t *platform, platform_sim_state_t *state)
{
    if (platform == 0 || state == 0)
        return PLATFORM_INVALID_ARGUMENT;
    platform->ops = &sim_ops;
    platform->context = state;
    return PLATFORM_OK;
}
