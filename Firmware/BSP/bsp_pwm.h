#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>
#include "bsp_status.h"

typedef enum { BSP_PWM_LEG_A = 0, BSP_PWM_LEG_B = 1 } bsp_pwm_leg_t;

/* Pure calculations. Duty is [0,1]; CCR may equal ARR+1 for 100% PWM. */
bsp_status_t bsp_pwm_duty_to_ccr(float duty, uint32_t arr, uint32_t *ccr);
bsp_status_t bsp_pwm_pulse_us_to_ccr(uint32_t pulse_us, uint32_t timer_hz,
                                      uint32_t prescaler, uint32_t arr, uint32_t *ccr);

/* Register writes only. No channel is started by these functions. */
bsp_status_t bsp_pwm_set_motor_leg(uint8_t joint, bsp_pwm_leg_t leg, float duty);
bsp_status_t bsp_pwm_set_servo_pulse_us(uint32_t pulse_us);

/* Explicit activation. Caller must establish electrical/mechanical safety first. */
bsp_status_t bsp_pwm_start_motor(uint8_t joint);
bsp_status_t bsp_pwm_stop_motor(uint8_t joint);
bsp_status_t bsp_pwm_start_servo(void);
bsp_status_t bsp_pwm_stop_servo(void);

#endif
