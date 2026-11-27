#ifndef UART_TEST_MAIN_H
#define UART_TEST_MAIN_H
#include <stdint.h>
#define USART1 ((void *)1)
#define DMA_CIRCULAR 1U
#define DMA_NORMAL 0U
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct {struct {unsigned Mode;} Init;} DMA_HandleTypeDef;
typedef struct {void *Instance;DMA_HandleTypeDef *hdmarx,*hdmatx;} UART_HandleTypeDef;
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_Abort(UART_HandleTypeDef *);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *,uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
