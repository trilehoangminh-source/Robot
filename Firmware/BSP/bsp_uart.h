#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_status.h"

#define BSP_UART_RX_DMA_CAPACITY 256U
#define BSP_UART_RX_RING_CAPACITY 512U
#define BSP_UART_TX_CAPACITY 256U

typedef struct {
    uint32_t rx_overflow_bytes;
    uint32_t hal_error;
    bool rx_active;
    bool tx_busy;
    bool fault;
} bsp_uart_status_t;

/* Explicit RX DMA start. No parser, CRC or protocol logic in this module.
 * Call from a task after CubeMX initializes USART1 and DMA2. */
void bsp_uart_set_notify(void (*notify)(void));
bsp_status_t bsp_uart_start_rx(void);
bsp_status_t bsp_uart_abort(void);
/* Single task consumer. RX callbacks are the sole ring producer. */
bsp_status_t bsp_uart_read(uint8_t *dst, uint16_t capacity, uint16_t *read_count);
/* Single task producer; data is copied to an internal DMA-safe TX buffer. */
bsp_status_t bsp_uart_write(const uint8_t *src, uint16_t length);
void bsp_uart_get_status(bsp_uart_status_t *status);

#endif
