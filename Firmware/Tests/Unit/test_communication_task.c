#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "communication_task.h"
static uint8_t dma[136];static uint16_t dma_length;static unsigned writes,aborts;static bsp_status_t outcome;
static bsp_status_t write_dma(const uint8_t *p,uint16_t n){if(outcome!=BSP_OK)return outcome;memcpy(dma,p,n);dma_length=n;writes++;return BSP_OK;}
static bsp_status_t abort_dma(void){aborts++;return BSP_OK;}
static bsp_status_t start_rx(void){return BSP_OK;}
static communication_task_t task;
static void init(void){communication_io_t io={write_dma,abort_dma,start_rx};shared_snapshot_t ready={0};shared_data_initialize();shared_snapshot_publish(&ready);assert(communication_initialize(&task,&io));outcome=BSP_OK;writes=aborts=0;}
static void feed(uint8_t id,uint8_t seq){frame_t f={0};uint8_t wire[136];size_t n;f.id=id;f.seq=seq;assert(frame_encode(&f,wire,sizeof wire,&n)==FRAME_OK);assert(communication_feed(&task,wire,n,10));}
static void accept(uint8_t seq){frame_t r={0};r.id=COMM_MSG_ACK;r.seq=seq;r.length=2;r.payload[0]=COMM_CMD_HOME;assert(shared_response_push(&r));}
int main(void){command_t c;frame_t f={0},decoded;uint8_t wire[136],saved[136];size_t n;unsigned i;uint32_t heartbeat;
 init();f.id=COMM_CMD_HOME;f.seq=255;assert(frame_encode(&f,wire,sizeof wire,&n)==FRAME_OK);
 for(i=0;i<n;i++)assert(communication_feed(&task,wire+i,1,10));assert(shared_command_pop(&c)&&c.seq==255);assert(task.pending&&writes==0);
 feed(COMM_CMD_HOME,255);assert(!shared_command_pop(&c));assert(task.diagnostics.duplicates==1);
 accept(255);communication_poll(&task,11,false,false,false);assert(writes==1);assert(frame_decode(dma,dma_length,&decoded)==FRAME_OK&&decoded.seq==255&&decoded.id==COMM_MSG_ACK);memcpy(saved,dma,dma_length);
 feed(COMM_CMD_HOME,255);communication_poll(&task,12,true,false,false);assert(writes==1&&!memcmp(saved,dma,dma_length));communication_poll(&task,13,false,false,false);assert(writes==2);
 feed(COMM_CMD_ENABLE,255);assert(task.diagnostics.conflicts==1);feed(COMM_CMD_HOME,0);assert(shared_command_pop(&c)&&c.seq==0);
 feed(COMM_CMD_STOP,1);assert(shared_stop_command_take(&c)&&c.id==COMM_CMD_STOP&&c.seq==1);assert(!shared_command_pop(&c));feed(COMM_CMD_HOME,0);assert(!shared_command_pop(&c));assert(task.response.id==COMM_MSG_NACK&&task.response.payload[1]==NACK_STATE);
 f.id=COMM_MSG_ACK;f.seq=1;f.length=2;f.payload[0]=COMM_CMD_STOP;shared_stop_response(&f);communication_poll(&task,14,false,false,false);feed(COMM_CMD_HOME,2);assert(shared_command_pop(&c)&&c.seq==2);
 init();feed(COMM_CMD_HEARTBEAT,22);assert(shared_heartbeat_take(&heartbeat)&&heartbeat==10);assert(!task.hn&&!task.valid&&!shared_command_pop(&c));
 c.id=COMM_CMD_HOME;for(i=0;i<SHARED_COMMAND_CAPACITY;i++)assert(shared_command_push(&c));feed(COMM_CMD_HOME,8);assert(task.response.payload[1]==NACK_BUFFER_FULL&&!task.pending);feed(COMM_CMD_STOP,9);assert(shared_stop_command_take(&c)&&!shared_command_pop(&c));
 init();f.id=COMM_CMD_HOME;f.seq=1;f.length=0;assert(frame_encode(&f,wire,sizeof wire,&n)==FRAME_OK);wire[n-1]^=1;assert(communication_feed(&task,wire,n,0));assert(task.parser.crc_errors==1&&!task.valid);wire[n-1]^=1;assert(communication_feed(&task,wire,3,1));communication_poll(&task,101,false,false,false);assert(task.parser.timeouts==1);assert(communication_feed(&task,wire,n,102));assert(task.pending);
 communication_poll(&task,103,false,false,true);assert(task.diagnostics.rx_overflows==1&&aborts==1&&!task.pending);assert(shared_stop_take());
 init();f.id=COMM_MSG_ACK;f.length=2;for(i=0;i<COMM_TX_HIGH_CAPACITY;i++)assert(communication_transmit(&task,&f));assert(!communication_transmit(&task,&f));outcome=BSP_BUSY;communication_poll(&task,1,false,false,false);assert(task.active_valid&&!task.inflight&&task.diagnostics.tx_busy==1);outcome=BSP_OK;communication_poll(&task,2,false,false,false);assert(task.inflight);communication_poll(&task,102,true,false,false);assert(aborts==1&&task.active_valid);communication_poll(&task,103,false,false,false);assert(writes==2);communication_poll(&task,104,true,true,false);assert(task.diagnostics.uart_errors==1);
 init();for(i=0;i<SHARED_RESPONSE_CAPACITY;i++)assert(shared_response_push(&f));assert(!shared_response_available());f.id=COMM_MSG_EVENT;f.length=5;f.payload[0]=EVENT_STOP_DONE;assert(shared_response_push(&f));for(i=0;i<SHARED_RESPONSE_CAPACITY;i++)assert(shared_response_pop(&decoded));assert(shared_response_pop(&decoded)&&decoded.id==COMM_MSG_EVENT);
 printf("communication task: PASS; static engine=%zu bytes\n",sizeof task);return 0;}
