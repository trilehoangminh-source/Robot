#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H
#include "protocol.h"
typedef struct {uint8_t id,seq,length,payload[COMM_MAX_PAYLOAD_BYTES];} frame_t;
typedef enum {FRAME_OK,FRAME_ARGUMENT,FRAME_LENGTH,FRAME_VERSION,FRAME_CRC,FRAME_SOF} frame_status_t;
frame_status_t frame_encode(const frame_t *f,uint8_t *out,size_t capacity,size_t *written);
frame_status_t frame_decode(const uint8_t *data,size_t length,frame_t *out);
#endif
