#include "communication_task.h"
#include <string.h>
static bool control_ready(void){shared_snapshot_t s;return shared_snapshot_read(&s)&&!s.startup_blocked;}
static bool same(const frame_t *a,const frame_t *b){return a->id==b->id&&a->seq==b->seq&&a->length==b->length&&memcmp(a->payload,b->payload,a->length)==0;}
static frame_t nack(const frame_t *f,uint8_t code){frame_t r={0};r.id=COMM_MSG_NACK;r.seq=f->seq;r.length=2;r.payload[0]=f->id;r.payload[1]=code;return r;}
bool communication_transmit(communication_task_t *t,const frame_t *f){bool low;
 if(!t||!f||f->length>COMM_MAX_PAYLOAD_BYTES)return false;
 low=f->id==COMM_MSG_JOINT_STATE||f->id==COMM_MSG_CONTROL_DEBUG;
 if(low){if(t->ln==COMM_TX_LOW_CAPACITY){t->diagnostics.tx_dropped++;shared_telemetry_transport_drop();return false;}t->low[(t->lt+t->ln)%COMM_TX_LOW_CAPACITY]=*f;t->ln++;}
 else{if(t->hn==COMM_TX_HIGH_CAPACITY){t->diagnostics.tx_dropped++;return false;}t->high[(t->ht+t->hn)%COMM_TX_HIGH_CAPACITY]=*f;t->hn++;}return true;
}
static void receive(void *ctx,const frame_t *f){communication_task_t *t=ctx;command_t c;frame_t r;uint8_t error=command_decode(f,&c);
 if(error){r=nack(f,error);(void)communication_transmit(t,&r);return;}
 if(f->id==COMM_CMD_HEARTBEAT){shared_heartbeat_publish(t->parser.last_ms);return;}
 if(f->id==COMM_CMD_STOP){
  if(t->stop_valid&&same(f,&t->stop_request)){t->diagnostics.duplicates++;if(!t->stop_pending)(void)communication_transmit(t,&t->stop_response);return;}
  /* STOP always reaches its safety mailbox even during sequence conflicts. */
  if(t->pending){t->response=nack(&t->request,NACK_STATE);t->pending=false;(void)communication_transmit(t,&t->response);}
  t->stop_valid=true;t->stop_request=*f;
  if(!control_ready()){shared_stop_request();t->stop_pending=false;t->stop_response=nack(f,NACK_BUSY);(void)communication_transmit(t,&t->stop_response);return;}
  t->stop_pending=true;shared_stop_command(f->seq);return;
 }
 if(t->valid&&f->seq==t->request.seq){if(same(f,&t->request)){t->diagnostics.duplicates++;if(!t->pending)(void)communication_transmit(t,&t->response);}else{t->diagnostics.conflicts++;r=nack(f,NACK_SEQUENCE);(void)communication_transmit(t,&r);}return;}
 /* Session never silently resets on idle/reconnect. Host retains last normal
  * sequence, or restarts MCU safely. STOP/heartbeat do not consume this sequence. */
 if(t->pending||t->stop_pending){r=nack(f,NACK_BUSY);(void)communication_transmit(t,&r);return;}
 if(t->valid&&f->seq!=(uint8_t)(t->request.seq+1U)&&!(t->stop_valid&&t->stop_request.seq==(uint8_t)(t->request.seq+1U)&&f->seq==(uint8_t)(t->stop_request.seq+1U))){t->diagnostics.conflicts++;r=nack(f,NACK_SEQUENCE);(void)communication_transmit(t,&r);return;}
 t->request=*f;t->valid=true;t->stop_valid=false;
 if(!control_ready()){
  telemetry_snapshot_t info={0};info.fw_major=1;
  if(f->id==COMM_CMD_PING)(void)telemetry_encode(COMM_MSG_PONG,f->seq,&info,&t->response);
  else if(f->id==COMM_CMD_GET_INFO)(void)telemetry_encode(COMM_MSG_ROBOT_INFO,f->seq,&info,&t->response);
  else t->response=nack(f,NACK_BUSY);
  (void)communication_transmit(t,&t->response);
 }else if(shared_command_push(&c)){t->pending=true;}else{t->diagnostics.queue_full++;t->response=nack(f,NACK_BUFFER_FULL);(void)communication_transmit(t,&t->response);}
}
bool communication_initialize(communication_task_t *t,const communication_io_t *io){if(!t||!io||!io->write||!io->abort||!io->start_rx)return false;memset(t,0,sizeof *t);t->io=*io;return parser_initialize(&t->parser,COMM_ACK_TIMEOUT_MS);}
bool communication_feed(communication_task_t *t,const uint8_t *p,size_t n,uint32_t now){if(!t||n>COMM_RX_BUDGET)return false;return parser_feed(&t->parser,p,n,now,receive,t);}
void communication_poll(communication_task_t *t,uint32_t now,bool busy,bool error,bool overflow){frame_t r;bsp_status_t result;
 if(!t)return;parser_expire(&t->parser,now);
 if(!control_ready()){
  if(t->pending){shared_stop_request();t->response=nack(&t->request,NACK_BUSY);t->pending=false;(void)communication_transmit(t,&t->response);}
  if(t->stop_pending){shared_stop_request();t->stop_response=nack(&t->stop_request,NACK_BUSY);t->stop_pending=false;(void)communication_transmit(t,&t->stop_response);}
 }
 if(error||overflow){if(error)t->diagnostics.uart_errors++;if(overflow)t->diagnostics.rx_overflows++;
  t->parser.used=0;shared_stop_request();
  if(t->pending){t->response=nack(&t->request,NACK_STATE);t->pending=false;(void)communication_transmit(t,&t->response);}
  if(t->io.abort()==BSP_OK){t->inflight=false;busy=false;(void)t->io.start_rx();}else return;
 }
 /* Never remove a manager response while the bounded TX ring is full. */
 if(t->hn<COMM_TX_HIGH_CAPACITY&&shared_stop_response_take(&r)){t->stop_response=r;t->stop_pending=false;(void)communication_transmit(t,&r);}
 if(t->hn<COMM_TX_HIGH_CAPACITY&&shared_response_pop(&r)){
  if(r.id==COMM_MSG_EVENT||r.id==COMM_MSG_FAULT){r.seq=t->event_seq++;(void)communication_transmit(t,&r);}
  else if(t->pending&&r.seq==t->request.seq){t->response=r;t->pending=false;(void)communication_transmit(t,&r);}
 }
 if(t->hn==0U&&t->ln<COMM_TX_LOW_CAPACITY&&shared_telemetry_pop(&r))(void)communication_transmit(t,&r);
 if(t->inflight){if(busy){if((uint32_t)(now-t->tx_started)>=COMM_ACK_TIMEOUT_MS){t->diagnostics.tx_timeouts++;if(t->io.abort()==BSP_OK){t->inflight=false;t->parser.used=0;(void)t->io.start_rx();}}return;}t->inflight=false;if(t->active.id==COMM_MSG_JOINT_STATE||t->active.id==COMM_MSG_CONTROL_DEBUG)shared_telemetry_complete();t->active_valid=false;}
 if(t->active_valid&&t->hn&&(t->active.id==COMM_MSG_JOINT_STATE||t->active.id==COMM_MSG_CONTROL_DEBUG)){t->active_valid=false;shared_telemetry_transport_drop();t->diagnostics.tx_dropped++;}
 if(!t->active_valid){if(t->hn){t->active=t->high[t->ht];t->ht=(t->ht+1U)%COMM_TX_HIGH_CAPACITY;t->hn--;}
  else if(t->ln){t->active=t->low[t->lt];t->lt=(t->lt+1U)%COMM_TX_LOW_CAPACITY;t->ln--;}else return;
  if(t->active.id==COMM_MSG_JOINT_STATE||t->active.id==COMM_MSG_CONTROL_DEBUG){if(t->active.length<4U||(uint32_t)(now-protocol_u32(t->active.payload))>COMM_TELEMETRY_MAX_AGE_MS){shared_telemetry_transport_drop();t->diagnostics.tx_dropped++;return;}t->active.seq=t->event_seq++;}
  if(frame_encode(&t->active,t->wire,sizeof t->wire,&t->wire_length)!=FRAME_OK){t->diagnostics.tx_errors++;return;}t->active_valid=true;}
 if((t->active.id==COMM_MSG_JOINT_STATE||t->active.id==COMM_MSG_CONTROL_DEBUG)&&(uint32_t)(now-protocol_u32(t->active.payload))>COMM_TELEMETRY_MAX_AGE_MS){t->active_valid=false;shared_telemetry_transport_drop();t->diagnostics.tx_dropped++;return;}
 result=t->io.write(t->wire,(uint16_t)t->wire_length);
 if(result==BSP_OK){t->inflight=true;t->tx_started=now;}else if(result==BSP_BUSY)t->diagnostics.tx_busy++;else t->diagnostics.tx_errors++;
}
#ifndef TASKS_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "bsp_timebase.h"
static TaskHandle_t comm_handle;
static void uart_notify(void){BaseType_t wake=pdFALSE;if(comm_handle&&xTaskGetSchedulerState()==taskSCHEDULER_RUNNING){vTaskNotifyGiveFromISR(comm_handle,&wake);portYIELD_FROM_ISR(wake);}}
void communication_task_run(void *argument){static communication_task_t state;static uint8_t chunk[64];uint32_t overflow=0;const communication_io_t io={bsp_uart_write,bsp_uart_abort,bsp_uart_start_rx};(void)argument;
 if(NVIC_GetPriority(USART1_IRQn)<configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY||NVIC_GetPriority(DMA2_Stream2_IRQn)<configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY||NVIC_GetPriority(DMA2_Stream7_IRQn)<configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY){vTaskSuspend(NULL);}
 (void)communication_initialize(&state,&io);comm_handle=xTaskGetCurrentTaskHandle();bsp_uart_set_notify(uart_notify);
 for(;;){bsp_uart_status_t status;unsigned budget=COMM_RX_BUDGET;uint16_t n;bsp_uart_get_status(&status);
  if(!status.rx_active&&!status.fault)(void)bsp_uart_start_rx();
  communication_poll(&state,bsp_timebase_now_ms(),status.tx_busy,status.fault,status.rx_overflow_bytes!=overflow);bsp_uart_get_status(&status);overflow=status.rx_overflow_bytes;
  while(budget){if(bsp_uart_read(chunk,sizeof chunk,&n)!=BSP_OK||!n)break;(void)communication_feed(&state,chunk,n,bsp_timebase_now_ms());budget-=n;}
  /* Finite wait also expires parser and TX watchdog without incoming IRQs. */
  (void)ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(1));
 }
}
#endif
