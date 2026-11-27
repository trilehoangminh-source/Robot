#include "bsp_uart.h"

#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

/* Default .bss is in normal SRAM (0x20000000), not CCM (0x10000000).
 * Verify placement in the Keil map if linker/scatter settings change. */
static void (*notify_task)(void);
void bsp_uart_set_notify(void (*notify)(void)){notify_task=notify;}
static uint8_t rx_dma[BSP_UART_RX_DMA_CAPACITY];
static uint8_t tx_dma[BSP_UART_TX_CAPACITY];
static uint8_t rx_ring[BSP_UART_RX_RING_CAPACITY];
static volatile uint16_t ring_head;
static volatile uint16_t ring_tail;
static uint16_t dma_position;
static bool full_captured;
static volatile uint32_t rx_overflow_bytes;
static volatile uint32_t hal_error;
static volatile bool rx_active;
static volatile bool tx_busy;
static volatile bool fault;

static bool dma_sram_buffer(const void *buffer, uint32_t length)
{
#ifdef BSP_UART_HOST_TEST
    (void)buffer;(void)length;return true;
#else
    uintptr_t start = (uintptr_t)buffer;
    /* STM32F407 DMA2 sees SRAM1/2, not CCM at 0x10000000. */
    return start >= 0x20000000UL && start < 0x2001C000UL &&
           length <= 0x2001C000UL - start;
#endif
}

static void ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((ring_head + 1U) % BSP_UART_RX_RING_CAPACITY);
    if (next == ring_tail) {
        ++rx_overflow_bytes; /* Drop newest; never overwrite unread bytes. */
        return;
    }
    rx_ring[ring_head] = byte;
    ring_head = next;
}

static void capture_rx(uint16_t size)
{
    uint16_t i;
    if (!rx_active || size > BSP_UART_RX_DMA_CAPACITY)
        return;
    if (size == BSP_UART_RX_DMA_CAPACITY && full_captured)
        return; /* Same boundary reported by both TC and IDLE. */
    if (size < BSP_UART_RX_DMA_CAPACITY)
        full_captured = false;
    if (size < dma_position) {
        for (i = dma_position; i < BSP_UART_RX_DMA_CAPACITY; ++i)
            ring_push(rx_dma[i]);
        dma_position = 0U;
    }
    for (i = dma_position; i < size; ++i)
        ring_push(rx_dma[i]);
    dma_position = size == BSP_UART_RX_DMA_CAPACITY ? 0U : size;
    if (size == BSP_UART_RX_DMA_CAPACITY)
        full_captured = true;
}

bsp_status_t bsp_uart_start_rx(void)
{
    if (huart1.Instance != USART1 || huart1.hdmarx == 0 || huart1.hdmatx == 0)
        return BSP_NOT_READY;
    if (huart1.hdmarx->Init.Mode != DMA_CIRCULAR ||
        huart1.hdmatx->Init.Mode != DMA_NORMAL)
        return BSP_NOT_READY;
    if (!dma_sram_buffer(rx_dma, sizeof rx_dma)) return BSP_NOT_READY;
    if (fault) return BSP_IO_ERROR;
    if (rx_active) return BSP_BUSY;
    ring_head = ring_tail = dma_position = 0U;
    full_captured = false;
    rx_overflow_bytes = 0U;
    rx_active = true;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma, BSP_UART_RX_DMA_CAPACITY) != HAL_OK) {
        rx_active = false;
        return BSP_IO_ERROR;
    }
    return BSP_OK;
}

bsp_status_t bsp_uart_abort(void)
{
    if (huart1.Instance != USART1) return BSP_NOT_READY;
    if (HAL_UART_Abort(&huart1) != HAL_OK) return BSP_IO_ERROR;
    rx_active = false;
    tx_busy = false;
    fault = false;
    hal_error = 0U;
    ring_head = ring_tail = dma_position = 0U;
    full_captured = false;
    return BSP_OK;
}

bsp_status_t bsp_uart_read(uint8_t *dst, uint16_t capacity, uint16_t *read_count)
{
    uint16_t n = 0U;
    if (dst == 0 || read_count == 0 || capacity == 0U)
        return BSP_INVALID_ARGUMENT;
    *read_count = 0U;
    if (fault) return BSP_IO_ERROR;
    if (!rx_active) return BSP_NOT_READY;
    while (n < capacity && ring_tail != ring_head) {
        dst[n++] = rx_ring[ring_tail];
        ring_tail = (uint16_t)((ring_tail + 1U) % BSP_UART_RX_RING_CAPACITY);
    }
    *read_count = n;
    return BSP_OK;
}

bsp_status_t bsp_uart_write(const uint8_t *src, uint16_t length)
{
    if (src == 0 || length == 0U || length > BSP_UART_TX_CAPACITY)
        return BSP_INVALID_ARGUMENT;
    if (huart1.Instance != USART1 || huart1.hdmatx == 0 ||
        huart1.hdmatx->Init.Mode != DMA_NORMAL)
        return BSP_NOT_READY;
    if (!dma_sram_buffer(tx_dma, sizeof tx_dma)) return BSP_NOT_READY;
    if (fault) return BSP_IO_ERROR;
    if (tx_busy) return BSP_BUSY;
    tx_busy = true;
    memcpy(tx_dma, src, length);
    if (HAL_UART_Transmit_DMA(&huart1, tx_dma, length) != HAL_OK) {
        tx_busy = false;
        return BSP_IO_ERROR;
    }
    return BSP_OK;
}

void bsp_uart_get_status(bsp_uart_status_t *status)
{
    if (status == 0) return;
    status->rx_overflow_bytes = rx_overflow_bytes;
    status->hal_error = hal_error;
    status->rx_active = rx_active;
    status->tx_busy = tx_busy;
    status->fault = fault;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart1) {
        capture_rx(size);
        if(notify_task)notify_task();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {tx_busy = false;if(notify_task)notify_task();}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        hal_error = HAL_UART_GetError(huart);
        fault = true;
        rx_active = false;
        tx_busy = false;
        if(notify_task)notify_task();
    }
}
