#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#define ROBOT_JOINT_COUNT 3U
#define ROBOT_J1 0U
#define ROBOT_J2 1U
#define ROBOT_J3 2U

/* The context's 80/127.5/147.5 mm geometry is preliminary, not calibrated. */
#define ROBOT_GEOMETRY_VERIFIED 0U
#define ROBOT_BASE_HEIGHT_MM_PRELIMINARY 80.0f
#define ROBOT_LINK_2_MM_PRELIMINARY 127.5f
#define ROBOT_LINK_3_MM_PRELIMINARY 147.5f

/* 7392 counts/rev is only a hypothesis (11 PPR * 168 * 4).
 * Zero means unknown. Motion control must reject any invalid calibration. */
#define ROBOT_ENCODER_CPR_VERIFIED 0U
#define ROBOT_ENCODER_CPR_UNKNOWN 0U
#define ROBOT_JOINT_LIMITS_VERIFIED 0U
#define ROBOT_HOME_OFFSETS_VERIFIED 0U
#define ROBOT_ENCODER_SIGNS_VERIFIED 0U

typedef struct {
    uint32_t encoder_cpr;
    int8_t encoder_sign;  /* Must be +1 or -1 after calibration; 0 means unknown. */
    int32_t home_count;
    float home_angle_rad;
    float min_angle_rad;
    float max_angle_rad;
    uint8_t calibration_valid;
    uint8_t limits_valid;
} robot_joint_calibration_t;

#endif
