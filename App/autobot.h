/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef AUTOBOT_H
#define AUTOBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI                               3.14159265359   //


/* ROBOT Parameter */
#define WHEEL_BASE                      0.15            /* [m] distance between front and rear wheel */
#define TRACK_WIDTH                     0.15            /* [m] distance between 2 wheel */
#define WHEEL_RADIUS                    0.03            /* [m] wheel's radius */

#define TURNING_RADIUS                  0.1435          /* [m] */
#define ROBOT_RADIUS                    0.220           /* [m] */

/* Encoder Parameter */
#define ENCODER_MIN                     -2147483648     /* [raw] */
#define ENCODER_MAX                     2147483648      /* [raw] */

/* Motor Parameter */
#define MOTOR_MAX_ROTATION              100                                         /* [rpm] */
#define MOTOR_MAX_SPEED                 2*PI*WHEEL_RADIUS*MOTOR_MAX_ROTATION/60     /* [m/s] */
#define MOTOR_MIN_SPEED                 -MOTOR_MAX_SPEED                            /* [m/s] */
#define ANGLR_MAX_SPEED                 (MOTOR_MAX_SPEED / TRACK_WIDTH)             /* [rad/s] */
#define ANGLR_MIN_SPEED                 -ANGLR_MAX_SPEED                            /* [rad/s] */


/*******************************************************************************
 * Global functions
 ******************************************************************************/
void setup();
void loop();

#ifdef __cplusplus
}
#endif

#endif /* AUTOBOT_H */
