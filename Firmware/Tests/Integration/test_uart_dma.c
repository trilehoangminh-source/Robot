#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "bsp_uart.h"
static DMA_HandleTypeDef rx={{DMA_CIRCULAR}},tx={{DMA_NORMAL}};
UART_HandleTypeDef huart1={USART1,&rx,&tx};
static uint8_t *receive_buffer,*transmit_buffer;static unsigned notifications;
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n){(void)h;assert(n==256);receive_buffer=p;return HAL_OK;}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n){(void)h;(void)n;transmit_buffer=p;return HAL_OK;}
int HAL_UART_Abort(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
uint32_t HAL_UART_GetError(UART_HandleTypeDef *h){(void)h;return 8;}
static void notify(void){notifications++;}
int main(void){uint8_t out[512],data[20];uint16_t n;unsigned i;bsp_uart_status_t s;
 bsp_uart_set_notify(notify);assert(bsp_uart_start_rx()==BSP_OK);for(i=0;i<256;i++)receive_buffer[i]=(uint8_t)i;
 HAL_UARTEx_RxEventCallback(&huart1,128);HAL_UARTEx_RxEventCallback(&huart1,256);HAL_UARTEx_RxEventCallback(&huart1,256);
 assert(bsp_uart_read(out,512,&n)==BSP_OK&&n==256);for(i=0;i<n;i++)assert(out[i]==(uint8_t)i);
 HAL_UARTEx_RxEventCallback(&huart1,20);assert(bsp_uart_read(out,512,&n)==BSP_OK&&n==20);for(i=0;i<n;i++)assert(out[i]==(uint8_t)i);
 HAL_UARTEx_RxEventCallback(&huart1,128);HAL_UARTEx_RxEventCallback(&huart1,256);HAL_UARTEx_RxEventCallback(&huart1,128);HAL_UARTEx_RxEventCallback(&huart1,256);HAL_UARTEx_RxEventCallback(&huart1,128);
 bsp_uart_get_status(&s);assert(s.rx_overflow_bytes>0);assert(bsp_uart_read(out,512,&n)==BSP_OK&&n==511);
 memset(data,42,sizeof data);assert(bsp_uart_write(data,sizeof data)==BSP_OK);memset(data,17,sizeof data);assert(bsp_uart_write(data,sizeof data)==BSP_BUSY);for(i=0;i<sizeof data;i++)assert(transmit_buffer[i]==42);
 HAL_UART_TxCpltCallback(&huart1);assert(bsp_uart_write(data,sizeof data)==BSP_OK);HAL_UART_ErrorCallback(&huart1);bsp_uart_get_status(&s);assert(s.fault&&s.hal_error==8);assert(bsp_uart_abort()==BSP_OK&&bsp_uart_start_rx()==BSP_OK);assert(notifications>0);
 puts("BSP DMA integration mock: PASS (wrap/overflow/callbacks/TX ownership)");return 0;}
