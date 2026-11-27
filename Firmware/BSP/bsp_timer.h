#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include "bsp_status.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

/* Callback runs in TIM6 IRQ; use only ISR-safe notification APIs. */
typedef void (*bsp_timer_control_callback_t)(void *context);

/* Requires a valid Control Task, running kernel and callback. No automatic start. */
bsp_status_t bsp_timer_start_control(osThreadId_t control_task,
                                      bsp_timer_control_callback_t callback,
                                      void *context);
bsp_status_t bsp_timer_stop_control(void);
/* Called from CubeMX HAL_TIM_PeriodElapsedCallback, after the TIM7 branch. */
void bsp_timer_on_period_elapsed(TIM_HandleTypeDef *htim);

#endif
