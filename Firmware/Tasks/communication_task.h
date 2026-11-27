#ifndef ROBOT_COMMUNICATION_TASK_H
#define ROBOT_COMMUNICATION_TASK_H
#include "parser.h"
#include "shared_data.h"
#include "bsp_uart.h"
#define COMM_TX_HIGH_CAPACITY 8U
#define COMM_TX_LOW_CAPACITY 2U
#define COMM_RX_BUDGET 256U
/* Single task owner. write copies into BSP DMA storage; abort must synchronously
 * stop DMA before returning OK. No heap and no UART waits in Control Task. */
typedef struct {
 bsp_status_t (*write)(const uint8_t *,uint16_t);
 bsp_status_t (*abort)(void);
 bsp_status_t (*start_rx)(void);
} communication_io_t;
typedef struct {uint32_t rx_overflows,uart_errors,tx_errors,tx_timeouts,tx_busy,tx_dropped,duplicates,conflicts,queue_full;} communication_diagnostics_t;
typedef struct {
 parser_t parser;communication_io_t io;communication_diagnostics_t diagnostics;
 frame_t high[COMM_TX_HIGH_CAPACITY],low[COMM_TX_LOW_CAPACITY],request,response,stop_request,stop_response,active;
 unsigned hn,ht,ln,lt;bool valid,pending,stop_valid,stop_pending,inflight,active_valid;
 uint8_t event_seq;uint8_t wire[136];size_t wire_length;uint32_t tx_started;
} communication_task_t;
bool communication_initialize(communication_task_t *task,const communication_io_t *io);
bool communication_feed(communication_task_t *task,const uint8_t *data,size_t size,uint32_t now);
void communication_poll(communication_task_t *task,uint32_t now,bool tx_busy,bool uart_error,bool overflow);
bool communication_transmit(communication_task_t *task,const frame_t *frame);
void communication_task_run(void *argument);
#endif
