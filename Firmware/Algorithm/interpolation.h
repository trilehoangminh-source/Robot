#ifndef ROBOT_INTERPOLATION_H
#define ROBOT_INTERPOLATION_H
#include <stdint.h>
typedef enum { INTERPOLATION_OK = 0, INTERPOLATION_INVALID } interpolation_status_t;
/* Time in milliseconds, position in radians. Clamp outside [start,end].
 * Timestamps are relative to one trajectory and must not wrap within a segment. */
interpolation_status_t interpolation_linear(uint32_t start_ms, uint32_t end_ms,
                                             float start_rad, float end_rad,
                                             uint32_t now_ms, float *position_rad);
#endif
