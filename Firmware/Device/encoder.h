#ifndef DEVICE_ENCODER_H
#define DEVICE_ENCODER_H
#include <stdbool.h>
#include "platform.h"
#include "robot_config.h"
#include "bsp_encoder.h"
typedef struct { int64_t count; float position_rad,velocity_rad_s; bool valid; } encoder_feedback_t;
bsp_status_t encoder_configure(uint8_t joint,const robot_joint_calibration_t *config);
bsp_status_t encoder_start(uint8_t joint);
bsp_status_t encoder_update(uint8_t joint,uint32_t elapsed_ms,encoder_feedback_t *feedback);
bsp_status_t encoder_reset_reference(uint8_t joint,float angle_rad);
/* Pure conversion; CPR and sign must be supplied, never assume 7392. */
bsp_status_t encoder_convert(int64_t count,int32_t delta,uint32_t elapsed_ms,
 const robot_joint_calibration_t *config,float *position,float *velocity);
bsp_status_t encoder_sample(uint8_t joint,uint32_t step_id,uint32_t dt_ms,platform_encoder_feedback_t *out);
#endif
