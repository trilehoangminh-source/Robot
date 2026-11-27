#ifndef ROBOT_HARDWARE_CONFIG_H
#define ROBOT_HARDWARE_CONFIG_H

#include <stdint.h>

/* CubeMX Firmware.ioc / generated main.c; pin-to-board wiring is unverified. */
#define HW_MCU_STM32F407VET6 1U
#define HW_HSE_HZ 8000000U
#define HW_CORE_CLOCK_HZ 168000000U
#define HW_APB1_TIMER_CLOCK_HZ 84000000U
#define HW_APB2_TIMER_CLOCK_HZ 168000000U
#define HW_JOINT_COUNT 3U

typedef struct {
    char port;
    uint8_t pin;
} hw_pin_t;

typedef struct {
    uint8_t encoder_timer;
    uint8_t encoder_counter_bits;
    hw_pin_t encoder_a;
    hw_pin_t encoder_b;
    uint8_t pwm_timer;
    uint8_t pwm_channel_a;
    uint8_t pwm_channel_b;
    hw_pin_t pwm_a;
    hw_pin_t pwm_b;
    hw_pin_t limit;
} hw_joint_io_t;

/* J1: TIM2 PA5/PA1, TIM1 CH1/CH2 PE9/PE11, PE2.
 * J2: TIM3 PC6/PC7, TIM1 CH3/CH4 PE13/PE14, PE3.
 * J3: TIM4 PD12/PD13, TIM12 CH1/CH2 PB14/PB15, PE4. */
static const hw_joint_io_t HW_JOINT_IO[HW_JOINT_COUNT] = {
    {2U, 32U, {'A', 5U}, {'A', 1U}, 1U, 1U, 2U, {'E', 9U}, {'E', 11U}, {'E', 2U}},
    {3U, 16U, {'C', 6U}, {'C', 7U}, 1U, 3U, 4U, {'E', 13U}, {'E', 14U}, {'E', 3U}},
    {4U, 16U, {'D', 12U}, {'D', 13U}, 12U, 1U, 2U, {'B', 14U}, {'B', 15U}, {'E', 4U}}
};

#define HW_TIM1_PWM_PRESCALER 0U
#define HW_TIM1_PWM_PERIOD 8399U
#define HW_TIM12_PWM_PRESCALER 0U
#define HW_TIM12_PWM_PERIOD 4199U
#define HW_MOTOR_PWM_HZ 20000U

#define HW_GRIPPER_TIMER 5U
#define HW_GRIPPER_CHANNEL 3U
#define HW_GRIPPER_PORT 'A'
#define HW_GRIPPER_PIN 2U
#define HW_GRIPPER_PRESCALER 83U
#define HW_GRIPPER_PERIOD 19999U
#define HW_GRIPPER_GENERATED_PULSE_US 1500U /* CubeMX initial CCR, not a safe open/close position. */

#define HW_CONTROL_TIMER 6U
#define HW_CONTROL_PRESCALER 83U
#define HW_CONTROL_PERIOD 999U
#define HW_HAL_TICK_TIMER 7U

#define HW_UART_INSTANCE 1U
#define HW_UART_TX_PORT 'A'
#define HW_UART_TX_PIN 9U
#define HW_UART_RX_PORT 'A'
#define HW_UART_RX_PIN 10U
#define HW_UART_BAUD 115200U
#define HW_UART_RX_DMA_CONTROLLER 2U
#define HW_UART_RX_DMA_STREAM 2U
#define HW_UART_RX_DMA_CHANNEL 4U
#define HW_UART_TX_DMA_CONTROLLER 2U
#define HW_UART_TX_DMA_STREAM 7U
#define HW_UART_TX_DMA_CHANNEL 4U
#define HW_FREERTOS_HEAP_BYTES 32768U
#define HW_FREERTOS_TICK_HZ 1000U
#define HW_FREERTOS_TASK_STACK_WORDS 512U
#define HW_CONTROL_TASK_CMSIS_PRIORITY 40U
#define HW_COMM_TASK_CMSIS_PRIORITY 32U
#define HW_ROBOT_TASK_CMSIS_PRIORITY 24U
#define HW_TELEMETRY_TASK_CMSIS_PRIORITY 16U
#define HW_FREERTOS_MAX_SYSCALL_IRQ_PRIORITY 5U
#define HW_CONTROL_IRQ_PRIORITY 5U
#define HW_UART_IRQ_PRIORITY 6U
#define HW_UART_DMA_IRQ_PRIORITY 6U
#define HW_HAL_TICK_IRQ_PRIORITY 15U
#define HW_PENDSV_IRQ_PRIORITY 15U
#define HW_SYSTICK_IRQ_PRIORITY 15U

/* GPIO inputs are pull-up. Active-high NC contact interpretation is unverified. */
#define HW_LIMIT_INPUT_PULLUP 1U
#define HW_LIMIT_POLARITY_VERIFIED 0U
#define HW_IWDG_ENABLED 0U

#endif
