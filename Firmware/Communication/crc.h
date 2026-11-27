#ifndef ROBOT_CRC_H
#define ROBOT_CRC_H
#include "protocol.h"
/* NULL is permitted only for zero length. */
uint16_t crc16_ccitt_false(const uint8_t *data,size_t length);
#endif
