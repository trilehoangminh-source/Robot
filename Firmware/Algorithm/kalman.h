#ifndef ROBOT_KALMAN_H
#define ROBOT_KALMAN_H
#include <stdbool.h>
typedef struct {float sa,r,p00,p01,p11,initial_velocity;} kalman_config_t;
typedef struct {kalman_config_t config;float q,w,p00,p01,p11;bool configured,initialized;} kalman_t;
bool kalman_initialize(kalman_t *k,const kalman_config_t *c);
bool kalman_configure(kalman_t *k,const kalman_config_t *c);
/* Reset requires a valid reference; first correction also initializes position. */
bool kalman_reset(kalman_t *k,float reference);
bool kalman_predict(kalman_t *k,float dt);
bool kalman_correct(kalman_t *k,float measurement);
/* Atomic predict+correct; caller explicitly uses predict alone for missing data.
 * Prediction is not permission to continue control after encoder faults. */
bool kalman_update(kalman_t *k,float dt,float measurement);
bool kalman_covariance_valid(float p00,float p01,float p11);
#endif
