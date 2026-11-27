#ifndef ROBOT_PARSER_H
#define ROBOT_PARSER_H
#include "frame.h"
typedef void (*parser_callback_t)(void *ctx,const frame_t *frame);
typedef struct {uint8_t buffer[136];size_t used;uint32_t last_ms,timeout_ms,frames,crc_errors,length_errors,version_errors,timeouts,noise;} parser_t;
/* Caller chooses inactivity timeout. Timeout discards incomplete candidate. */
bool parser_initialize(parser_t *p,uint32_t timeout_ms);
void parser_expire(parser_t *p,uint32_t now_ms);
bool parser_feed(parser_t *p,const uint8_t *data,size_t length,uint32_t now_ms,parser_callback_t callback,void *context);
#endif
