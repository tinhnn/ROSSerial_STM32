/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/* select motor control board */
#if defined(USING_PCA9685_BOARD)
#include "PCA9685/PCA9685.h"
#elif defined(USING_DFR0592_BOARD)
#include "DFR0592/DFR0592.h"
#elif defined(USING_NUCLEO_BOARD)
#include "nucleo/nucleo_board.h"
#endif



/* Motor Parameter */


/* motor define */
#define MOTOR_FRONT_LEFT                0
#define MOTOR_FRONT_RIGHT               1
#define MOTOR_REAR_LEFT                 2
#define MOTOR_REAR_RIGHT                3
#define MOTOR_ID_MAX                    4

#define MAX_DUTY_CYCLE                  1       /* default: 100 */

/* control board define */
#define BOARD_ADDRESS                   PCA9685_BOARD_ADDRESS

struct robot_param
{
    /* chassis param */
    float wheel_base;
    float track_width;
    float wheel_radius;
    /* motor param */
    uint32_t motor_max_rotation;
    double motor_max_speed;
    double motor_min_speed;
    double anglr_max_speed;
    double anglr_min_speed;
};

class motor_control
{
public:
    motor_control();
    void init(struct robot_param param);
    bool write_velocity(float linear_x, float angular_z);

private:
#if defined(USING_PCA9685_BOARD)
    PCA9685 _motor_drive;
#elif defined(USING_DFR0592_BOARD)
    DFR0592 _motor_drive;
#elif defined(USING_NUCLEO_BOARD)
    nucleo  _motor_drive;
#endif
    struct robot_param _robot_param;
    DCMotor *motor[MOTOR_ID_MAX];

    bool write_duty2HW(float left_duty, float right_duty);
};


#endif /* MOTOR_DRIVER_H */
