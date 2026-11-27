#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_timebase.h"

static uint32_t fake_tick;
uint32_t bsp_timebase_host_tick(void) { return fake_tick; }

int main(void)
{
    uint32_t ccr = 99U;
    int32_t delta = 99;

    assert(bsp_pwm_duty_to_ccr(0.0f, 8399U, &ccr) == BSP_OK && ccr == 0U);
    assert(bsp_pwm_duty_to_ccr(0.5f, 8399U, &ccr) == BSP_OK && ccr == 4200U);
    assert(bsp_pwm_duty_to_ccr(1.0f, 8399U, &ccr) == BSP_OK && ccr == 8400U);
    assert(bsp_pwm_duty_to_ccr(0.5f, 4199U, &ccr) == BSP_OK && ccr == 2100U);
    assert(bsp_pwm_duty_to_ccr(-0.1f, 8399U, &ccr) == BSP_INVALID_ARGUMENT);
    assert(bsp_pwm_duty_to_ccr(1.1f, 8399U, &ccr) == BSP_INVALID_ARGUMENT);
    assert(bsp_pwm_duty_to_ccr(NAN, 8399U, &ccr) == BSP_INVALID_ARGUMENT);
    assert(bsp_pwm_duty_to_ccr(1.0f, UINT32_MAX, &ccr) == BSP_INVALID_ARGUMENT);
    assert(bsp_pwm_duty_to_ccr(0.5f, 8399U, 0) == BSP_INVALID_ARGUMENT);
    assert(bsp_pwm_pulse_us_to_ccr(1500U, 84000000U, 83U, 19999U, &ccr) == BSP_OK && ccr == 1500U);
    assert(bsp_pwm_pulse_us_to_ccr(20000U, 84000000U, 83U, 19999U, &ccr) == BSP_OK && ccr == 20000U);
    assert(bsp_pwm_pulse_us_to_ccr(20001U, 84000000U, 83U, 19999U, &ccr) == BSP_INVALID_ARGUMENT);

    assert(bsp_encoder_delta(65534U, 1U, 16U, &delta) == BSP_OK && delta == 3);
    assert(bsp_encoder_delta(1U, 65534U, 16U, &delta) == BSP_OK && delta == -3);
    assert(bsp_encoder_delta(0xFFFFFFFEU, 1U, 32U, &delta) == BSP_OK && delta == 3);
    assert(bsp_encoder_delta(1U, 0xFFFFFFFEU, 32U, &delta) == BSP_OK && delta == -3);
    assert(bsp_encoder_delta(0U, 0x8000U, 16U, &delta) == BSP_OVERFLOW_AMBIGUOUS);
    assert(bsp_encoder_delta(0U, 0x80000000U, 32U, &delta) == BSP_OVERFLOW_AMBIGUOUS);
    assert(bsp_encoder_delta(0U, 0x10000U, 16U, &delta) == BSP_INVALID_ARGUMENT);
    assert(bsp_encoder_delta(0U, 1U, 24U, &delta) == BSP_INVALID_ARGUMENT);

    assert(bsp_timebase_elapsed_ms(100U, 110U) == 10U);
    assert(bsp_timebase_elapsed_ms(UINT32_MAX - 2U, 2U) == 5U);
    fake_tick = 2U;
    assert(bsp_timebase_elapsed_since_ms(UINT32_MAX - 2U) == 5U);

    puts("bsp_math: PASS");
    return 0;
}
