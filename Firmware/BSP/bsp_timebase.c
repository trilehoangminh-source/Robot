#include "bsp_timebase.h"

#ifdef BSP_HOST_TEST
extern uint32_t bsp_timebase_host_tick(void);
#else
#include "stm32f4xx_hal.h"
#endif

uint32_t bsp_timebase_now_ms(void)
{
#ifdef BSP_HOST_TEST
    return bsp_timebase_host_tick();
#else
    return HAL_GetTick(); /* TIM7 HAL timebase; never reconfigure TIM7 here. */
#endif
}

uint32_t bsp_timebase_elapsed_ms(uint32_t start_ms, uint32_t end_ms)
{
    return end_ms - start_ms;
}

uint32_t bsp_timebase_elapsed_since_ms(uint32_t start_ms)
{
    return bsp_timebase_elapsed_ms(start_ms, bsp_timebase_now_ms());
}
