#include "bsp_pwm.h"

#ifndef BSP_HOST_TEST
#include "hardware_config.h"
#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
static uint8_t servo_pulse_configured;
#endif

bsp_status_t bsp_pwm_duty_to_ccr(float duty, uint32_t arr, uint32_t *ccr)
{
    /* TIM1 and TIM12 PWM periods are 16-bit; keep float scaling exact here. */
    if (ccr == 0 || arr > UINT16_MAX || !(duty >= 0.0f && duty <= 1.0f))
        return BSP_INVALID_ARGUMENT;
    *ccr = (uint32_t)(duty * (float)(arr + 1U) + 0.5f);
    return BSP_OK;
}

bsp_status_t bsp_pwm_pulse_us_to_ccr(uint32_t pulse_us, uint32_t timer_hz,
                                      uint32_t prescaler, uint32_t arr, uint32_t *ccr)
{
    uint64_t numerator;
    uint64_t denominator;
    uint64_t ticks;
    if (ccr == 0 || timer_hz == 0U || prescaler == UINT32_MAX || arr == UINT32_MAX)
        return BSP_INVALID_ARGUMENT;
    numerator = (uint64_t)pulse_us * timer_hz;
    denominator = (uint64_t)(prescaler + 1U) * 1000000ULL;
    ticks = (numerator + denominator / 2U) / denominator;
    if (ticks > (uint64_t)arr + 1U)
        return BSP_INVALID_ARGUMENT;
    *ccr = (uint32_t)ticks;
    return BSP_OK;
}

#ifndef BSP_HOST_TEST
static bsp_status_t motor_channels(uint8_t joint, TIM_HandleTypeDef **timer,
                                    uint32_t *a, uint32_t *b)
{
    if (timer == 0 || a == 0 || b == 0 || joint >= HW_JOINT_COUNT)
        return BSP_INVALID_ARGUMENT;
    if (joint == 0U) {
        *timer = &htim1; *a = TIM_CHANNEL_1; *b = TIM_CHANNEL_2;
    } else if (joint == 1U) {
        *timer = &htim1; *a = TIM_CHANNEL_3; *b = TIM_CHANNEL_4;
    } else {
        *timer = &htim12; *a = TIM_CHANNEL_1; *b = TIM_CHANNEL_2;
    }
    return BSP_OK;
}

bsp_status_t bsp_pwm_set_motor_leg(uint8_t joint, bsp_pwm_leg_t leg, float duty)
{
    TIM_HandleTypeDef *timer;
    uint32_t a, b, ccr;
    if (leg != BSP_PWM_LEG_A && leg != BSP_PWM_LEG_B)
        return BSP_INVALID_ARGUMENT;
    if (motor_channels(joint, &timer, &a, &b) != BSP_OK ||
        bsp_pwm_duty_to_ccr(duty, joint == 2U ? HW_TIM12_PWM_PERIOD : HW_TIM1_PWM_PERIOD,
                            &ccr) != BSP_OK)
        return BSP_INVALID_ARGUMENT;
    if (timer->Instance == 0)
        return BSP_NOT_READY;
    __HAL_TIM_SET_COMPARE(timer, leg == BSP_PWM_LEG_A ? a : b, ccr);
    return BSP_OK;
}

bsp_status_t bsp_pwm_set_servo_pulse_us(uint32_t pulse_us)
{
    uint32_t ccr;
    if (bsp_pwm_pulse_us_to_ccr(pulse_us, HW_APB1_TIMER_CLOCK_HZ,
                                 HW_GRIPPER_PRESCALER, HW_GRIPPER_PERIOD, &ccr) != BSP_OK)
        return BSP_INVALID_ARGUMENT;
    if (htim5.Instance == 0)
        return BSP_NOT_READY;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ccr);
    servo_pulse_configured = 1U;
    return BSP_OK;
}

bsp_status_t bsp_pwm_start_motor(uint8_t joint)
{
    TIM_HandleTypeDef *timer;
    uint32_t a, b;
    if (motor_channels(joint, &timer, &a, &b) != BSP_OK)
        return BSP_INVALID_ARGUMENT;
    if (timer->Instance == 0)
        return BSP_NOT_READY;
    __HAL_TIM_SET_COMPARE(timer, a, 0U);
    __HAL_TIM_SET_COMPARE(timer, b, 0U);
    if (HAL_TIM_PWM_Start(timer, a) != HAL_OK)
        return BSP_IO_ERROR;
    if (HAL_TIM_PWM_Start(timer, b) != HAL_OK) {
        (void)HAL_TIM_PWM_Stop(timer, a);
        return BSP_IO_ERROR;
    }
    return BSP_OK;
}

bsp_status_t bsp_pwm_stop_motor(uint8_t joint)
{
    TIM_HandleTypeDef *timer;
    uint32_t a, b;
    HAL_StatusTypeDef first, second;
    if (motor_channels(joint, &timer, &a, &b) != BSP_OK)
        return BSP_INVALID_ARGUMENT;
    if (timer->Instance == 0)
        return BSP_NOT_READY;
    __HAL_TIM_SET_COMPARE(timer, a, 0U);
    __HAL_TIM_SET_COMPARE(timer, b, 0U);
    first = HAL_TIM_PWM_Stop(timer, a);
    second = HAL_TIM_PWM_Stop(timer, b);
    return first == HAL_OK && second == HAL_OK ? BSP_OK : BSP_IO_ERROR;
}

bsp_status_t bsp_pwm_start_servo(void)
{
    if (htim5.Instance == 0 || !servo_pulse_configured)
        return BSP_NOT_READY;
    return HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) == HAL_OK ? BSP_OK : BSP_IO_ERROR;
}

bsp_status_t bsp_pwm_stop_servo(void)
{
    if (htim5.Instance == 0)
        return BSP_NOT_READY;
    return HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3) == HAL_OK ? BSP_OK : BSP_IO_ERROR;
}
#endif
