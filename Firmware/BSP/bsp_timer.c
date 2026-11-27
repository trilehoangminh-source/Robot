#include "bsp_timer.h"

extern TIM_HandleTypeDef htim6;

static volatile uint8_t running;
static bsp_timer_control_callback_t control_callback;
static void *control_context;

bsp_status_t bsp_timer_start_control(osThreadId_t control_task,
                                      bsp_timer_control_callback_t callback,
                                      void *context)
{
    osThreadState_t state;
    if (control_task == 0 || callback == 0)
        return BSP_INVALID_ARGUMENT;
    if (running) return BSP_BUSY;
    if (htim6.Instance != TIM6 || osKernelGetState() != osKernelRunning)
        return BSP_NOT_READY;
    state = osThreadGetState(control_task);
    if (state == osThreadError || state == osThreadInactive || state == osThreadTerminated)
        return BSP_NOT_READY;
    control_callback = callback;
    control_context = context;
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    running = 1U;
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        running = 0U;
        control_callback = 0;
        control_context = 0;
        return BSP_IO_ERROR;
    }
    return BSP_OK;
}

bsp_status_t bsp_timer_stop_control(void)
{
    if (!running) return BSP_NOT_READY;
    running = 0U;
    if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK)
        return BSP_IO_ERROR;
    control_callback = 0;
    control_context = 0;
    return BSP_OK;
}

void bsp_timer_on_period_elapsed(TIM_HandleTypeDef *htim)
{
    if (htim != 0 && htim->Instance == TIM6 && running && control_callback != 0)
        control_callback(control_context);
}
