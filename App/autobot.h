/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef AUTOBOT_H
#define AUTOBOT_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Autobot Spec define
 ******************************************************************************/
#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.287           // meter
#define TURNING_RADIUS                   0.1435          // meter
#define ROBOT_RADIUS                     0.220           // meter
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw
#define MOTOR_ROT_MAX                    77              // rpm
#define PI                               3.14159265359   //

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * PI * MOTOR_ROT_MAX / 60) // m/s
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void setup();
void loop();

#ifdef __cplusplus
}
#endif

#endif /* AUTOBOT_H */
