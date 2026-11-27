#include "bsp_gpio.h"

#include "hardware_config.h"
#include "main.h"

bsp_status_t bsp_gpio_read_limit_raw(uint8_t joint, bool *pin_high)
{
    uint16_t pin;
    if (pin_high == 0 || joint >= HW_JOINT_COUNT)
        return BSP_INVALID_ARGUMENT;
    if ((RCC->AHB1ENR & RCC_AHB1ENR_GPIOEEN) == 0U)
        return BSP_NOT_READY;
    pin = joint == 0U ? J1_LIMIT_Pin : (joint == 1U ? J2_LIMIT_Pin : J3_LIMIT_Pin);
    *pin_high = HAL_GPIO_ReadPin(GPIOE, pin) == GPIO_PIN_SET;
    return BSP_OK;
}
