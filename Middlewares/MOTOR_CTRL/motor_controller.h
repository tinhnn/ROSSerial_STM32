/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/* select motor control board */
#define USING_PCA9685_BOARD

#if defined(USING_PCA9685_BOARD)
#include "PCA9685/PCA9685.h"
#elif defined(USING_DFR0592_BOARD)
#include "DFR0592/DFR0592.h"
#elif defined(USING_NUCLEO_BOARD)
#include "nucleo/nucleo_board.h"
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

#define MAX_ANGULAR_VELOCITY            (MOTOR_MAX_SPEED / TRACK_WIDTH)             /* [rad/s] */
#define MIN_ANGULAR_VELOCITY            -MAX_ANGULAR_VELOCITY                       /* [rad/s] */


/* motor define */
#define MOTOR_FRONT_LEFT                0
#define MOTOR_FRONT_RIGHT               1
#define MOTOR_REAR_LEFT                 2
#define MOTOR_REAR_RIGHT                3
#define MOTOR_ID_MAX                    4

#define MAX_DUTY_CYCLE                  100     /* default: 100 */

/* control board define */
#define BOARD_ADDRESS                   PCA9685_BOARD_ADDRESS

class motor_control
{
public:
    motor_control();
    void init(void);
    bool write_velocity(float linear_x, float angular_z);

private:
#if defined(USING_PCA9685_BOARD)
    PCA9685 _motor_drive;
#elif defined(USING_DFR0592_BOARD)
    DFR0592 _motor_drive;
#elif defined(USING_NUCLEO_BOARD)
    nucleo  _motor_drive;
#endif
    DCMotor *motor[MOTOR_ID_MAX];
    uint8_t _left_wheel_id;
    uint8_t _right_wheel_id;

    bool write_duty2HW(float left_duty, float right_duty);
};


#endif /* MOTOR_DRIVER_H */
