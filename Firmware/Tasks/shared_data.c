#include "shared_data.h"
#include <string.h>
#ifndef TASKS_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
#define LOCK() taskENTER_CRITICAL()
#define UNLOCK() taskEXIT_CRITICAL()
#else
/* Native tests exercise sequential logic only, never concurrency. */
#define LOCK() ((void)0)
#define UNLOCK() ((void)0)
#endif
static bool stop_command_valid,stop_reply_valid;
static uint8_t stop_seq;
static frame_t stop_reply,events[5];
static bool event_valid[5];
static command_t commands[SHARED_COMMAND_CAPACITY];
static frame_t responses[SHARED_RESPONSE_CAPACITY];
static unsigned ch,ct,cn,rh,rt,rn;
static bool stop,heartbeat_valid,snapshot_valid;
static uint32_t heartbeat,dropped;
static shared_snapshot_t latest;
static shared_application_diagnostics_t application;
static shared_telemetry_diagnostics_t telemetry_stats;
static frame_t telemetry_frames[2];
static bool telemetry_pending[2];
void shared_data_initialize(void){memset(&application,0,sizeof application);memset(&telemetry_stats,0,sizeof telemetry_stats);memset(telemetry_pending,0,sizeof telemetry_pending);stop_command_valid=stop_reply_valid=false;memset(event_valid,0,sizeof event_valid);ch=ct=cn=rh=rt=rn=0;stop=heartbeat_valid=snapshot_valid=false;heartbeat=dropped=0;memset(&latest,0,sizeof latest);}
bool shared_command_push(const command_t *c){bool ok=false;if(!c)return false;if(c->id==COMM_CMD_STOP){shared_stop_request();return true;}if(c->id==COMM_CMD_HEARTBEAT)return false;
 LOCK();if(!stop&&cn<SHARED_COMMAND_CAPACITY){commands[ch]=*c;ch=(ch+1U)%SHARED_COMMAND_CAPACITY;cn++;ok=true;}UNLOCK();return ok;}
bool shared_command_pop(command_t *c){bool ok=false;if(!c)return false;LOCK();if(!stop&&cn){*c=commands[ct];ct=(ct+1U)%SHARED_COMMAND_CAPACITY;cn--;ok=true;}UNLOCK();return ok;}
void shared_stop_request(void){LOCK();stop=true;ch=ct=cn=0;UNLOCK();}
bool shared_stop_take(void){bool pending;LOCK();pending=stop;stop=false;UNLOCK();return pending;}
void shared_heartbeat_publish(uint32_t now){LOCK();heartbeat=now;heartbeat_valid=true;UNLOCK();}
bool shared_heartbeat_take(uint32_t *out){bool ok=false;if(!out)return false;LOCK();if(heartbeat_valid){*out=heartbeat;heartbeat_valid=false;ok=true;}UNLOCK();return ok;}
bool shared_response_push(const frame_t *f){bool ok=false;if(!f)return false;LOCK();if((f->id==COMM_MSG_EVENT&&f->length==5U&&f->payload[0]>=1U&&f->payload[0]<=4U)||f->id==COMM_MSG_FAULT){unsigned slot=f->id==COMM_MSG_FAULT?0U:f->payload[0];if(event_valid[slot])dropped++;events[slot]=*f;event_valid[slot]=true;ok=true;}else if(rn<SHARED_RESPONSE_CAPACITY){responses[rh]=*f;rh=(rh+1U)%SHARED_RESPONSE_CAPACITY;rn++;ok=true;}else dropped++;UNLOCK();return ok;}
bool shared_response_pop(frame_t *f){bool ok=false;if(!f)return false;LOCK();if(rn){*f=responses[rt];rt=(rt+1U)%SHARED_RESPONSE_CAPACITY;rn--;ok=true;}else{unsigned i;for(i=0;i<5U;i++)if(event_valid[i]){*f=events[i];event_valid[i]=false;ok=true;break;}}UNLOCK();return ok;}
uint32_t shared_response_dropped(void){uint32_t n;LOCK();n=dropped;UNLOCK();return n;}
void shared_snapshot_publish(const shared_snapshot_t *s){if(!s)return;LOCK();latest=*s;snapshot_valid=true;UNLOCK();}
bool shared_snapshot_read(shared_snapshot_t *s){bool ok=false;if(!s)return false;LOCK();if(snapshot_valid){*s=latest;ok=true;}UNLOCK();return ok;}

void shared_stop_command(uint8_t seq){LOCK();stop_seq=seq;stop_command_valid=true;stop=true;ch=ct=cn=0;UNLOCK();}
bool shared_stop_command_take(command_t *c){bool ok=false;LOCK();if(stop_command_valid){memset(c,0,sizeof *c);c->id=COMM_CMD_STOP;c->seq=stop_seq;stop_command_valid=false;stop=false;ok=true;}UNLOCK();return ok;}
void shared_stop_response(const frame_t *f){LOCK();stop_reply=*f;stop_reply_valid=true;UNLOCK();}
bool shared_stop_response_take(frame_t *f){bool ok=false;LOCK();if(stop_reply_valid){*f=stop_reply;stop_reply_valid=false;ok=true;}UNLOCK();return ok;}
bool shared_response_available(void){bool ok;LOCK();ok=rn<SHARED_RESPONSE_CAPACITY;UNLOCK();return ok;}

bool shared_application_request(uint32_t now){bool fresh;LOCK();application.requested++;fresh=!application.pending;if(fresh){application.pending=true;application.last_request_ms=now;}else application.pending_collisions++;UNLOCK();return fresh;}
bool shared_application_take(uint32_t *now){bool ok=false;if(!now)return false;LOCK();if(application.pending){*now=application.last_request_ms;application.pending=false;application.taken++;ok=true;}UNLOCK();return ok;}
void shared_application_diagnostics(shared_application_diagnostics_t *out){if(!out)return;LOCK();*out=application;UNLOCK();}
bool shared_telemetry_push(const frame_t *f){unsigned slot;if(!f||(f->id!=COMM_MSG_JOINT_STATE&&f->id!=COMM_MSG_CONTROL_DEBUG))return false;slot=f->id==COMM_MSG_JOINT_STATE?0U:1U;LOCK();if(telemetry_pending[slot])telemetry_stats.replaced++;telemetry_frames[slot]=*f;telemetry_pending[slot]=true;telemetry_stats.enqueued++;UNLOCK();return true;}
bool shared_telemetry_pop(frame_t *f){unsigned i;bool ok=false;if(!f)return false;LOCK();for(i=0;i<2U;i++)if(telemetry_pending[i]){*f=telemetry_frames[i];telemetry_pending[i]=false;ok=true;break;}UNLOCK();return ok;}
void shared_telemetry_complete(void){LOCK();telemetry_stats.completed++;UNLOCK();}
void shared_telemetry_transport_drop(void){LOCK();telemetry_stats.transport_dropped++;UNLOCK();}
void shared_telemetry_diagnostics(shared_telemetry_diagnostics_t *out){if(!out)return;LOCK();*out=telemetry_stats;UNLOCK();}
