#include "bsp_encoder.h"

#ifndef BSP_HOST_TEST
#include "hardware_config.h"
#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static uint32_t previous_count[HW_JOINT_COUNT];
static uint8_t started[HW_JOINT_COUNT];
#endif

bsp_status_t bsp_encoder_delta(uint32_t previous, uint32_t current,
                                uint8_t counter_bits, int32_t *delta)
{
    uint32_t mask, difference, half;
    if (delta == 0 || (counter_bits != 16U && counter_bits != 32U))
        return BSP_INVALID_ARGUMENT;
    mask = counter_bits == 32U ? UINT32_MAX : 0xFFFFU;
    if ((previous & ~mask) != 0U || (current & ~mask) != 0U)
        return BSP_INVALID_ARGUMENT;
    difference = (current - previous) & mask;
    half = counter_bits == 32U ? 0x80000000U : 0x8000U;
    if (difference == half)
        return BSP_OVERFLOW_AMBIGUOUS;
    if (difference < half)
        *delta = (int32_t)difference;
    else
        *delta = -(int32_t)((uint64_t)mask + 1U - difference);
    return BSP_OK;
}

#ifndef BSP_HOST_TEST
static TIM_HandleTypeDef *encoder_timer(uint8_t joint)
{
    if (joint == 0U) return &htim2;
    if (joint == 1U) return &htim3;
    if (joint == 2U) return &htim4;
    return 0;
}

bsp_status_t bsp_encoder_start(uint8_t joint)
{
    TIM_HandleTypeDef *timer = encoder_timer(joint);
    if (timer == 0) return BSP_INVALID_ARGUMENT;
    if (timer->Instance == 0) return BSP_NOT_READY;
    if (started[joint]) return BSP_BUSY;
    if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) != HAL_OK)
        return BSP_IO_ERROR;
    previous_count[joint] = __HAL_TIM_GET_COUNTER(timer);
    started[joint] = 1U;
    return BSP_OK;
}

bsp_status_t bsp_encoder_stop(uint8_t joint)
{
    TIM_HandleTypeDef *timer = encoder_timer(joint);
    if (timer == 0) return BSP_INVALID_ARGUMENT;
    if (!started[joint]) return BSP_NOT_READY;
    if (HAL_TIM_Encoder_Stop(timer, TIM_CHANNEL_ALL) != HAL_OK)
        return BSP_IO_ERROR;
    started[joint] = 0U;
    return BSP_OK;
}

bsp_status_t bsp_encoder_read_raw(uint8_t joint, uint32_t *raw_count)
{
    TIM_HandleTypeDef *timer = encoder_timer(joint);
    if (timer == 0 || raw_count == 0) return BSP_INVALID_ARGUMENT;
    if (!started[joint]) return BSP_NOT_READY;
    *raw_count = __HAL_TIM_GET_COUNTER(timer) &
                 (joint == 0U ? UINT32_MAX : 0xFFFFU);
    return BSP_OK;
}

bsp_status_t bsp_encoder_sample_delta(uint8_t joint, int32_t *delta)
{
    uint32_t raw;
    bsp_status_t status;
    if (delta == 0) return BSP_INVALID_ARGUMENT;
    status = bsp_encoder_read_raw(joint, &raw);
    if (status != BSP_OK) return status;
    status = bsp_encoder_delta(previous_count[joint], raw,
                               HW_JOINT_IO[joint].encoder_counter_bits, delta);
    if (status == BSP_OK) previous_count[joint] = raw;
    return status;
}

bsp_status_t bsp_encoder_resync(uint8_t joint)
{
    uint32_t raw;
    bsp_status_t status = bsp_encoder_read_raw(joint, &raw);
    if (status == BSP_OK) previous_count[joint] = raw;
    return status;
}
#endif
