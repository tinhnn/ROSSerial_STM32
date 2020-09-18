/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "motor_controller.h"

#define LEFT        0
#define RIGHT       1
#define PI          3.14159265359
#define constrain(val,low,high) ((val)<=(low)?(low):((val)>=(high)?(high):(val)))

motor_control::motor_control()
              :_motor_drive(0x60)
{
    //TODO
}

void motor_control::init(struct robot_param param)
{
    //
    _robot_param = param;

    /* initial board */
    _motor_drive.init(1000);        /* PWM freq = 1kHz */
    // Get motor pin control
    motor[MOTOR_FRONT_LEFT]     = _motor_drive.getMotor(MOTOR_FRONT_LEFT + 1);      // motor 1
    motor[MOTOR_FRONT_RIGHT]    = _motor_drive.getMotor(MOTOR_FRONT_RIGHT + 1);     // motor 2
    motor[MOTOR_REAR_LEFT]      = _motor_drive.getMotor(MOTOR_REAR_LEFT + 1);       // motor 3
    motor[MOTOR_REAR_RIGHT]     = _motor_drive.getMotor(MOTOR_REAR_RIGHT + 1);      // motor 4
}

bool motor_control::write_velocity(float linear_x, float angular_z)
{
    bool ret;
    double wheel_velocity[2];

    wheel_velocity[LEFT]  = linear_x - angular_z * _robot_param.track_width / 2;
    wheel_velocity[RIGHT] = linear_x + angular_z * _robot_param.track_width / 2;

    wheel_velocity[LEFT]  = constrain(wheel_velocity[LEFT],  _robot_param.motor_min_speed, _robot_param.motor_max_speed);
    wheel_velocity[RIGHT] = constrain(wheel_velocity[RIGHT], _robot_param.motor_min_speed, _robot_param.motor_max_speed);

    /* Convert to duty */
    float left_duty  = (float)(wheel_velocity[LEFT]  / _robot_param.motor_max_speed * MAX_DUTY_CYCLE);
    float right_duty = (float)(wheel_velocity[RIGHT] / _robot_param.motor_max_speed * MAX_DUTY_CYCLE);

    /* write duty to HW */
    ret = write_duty2HW(left_duty, right_duty);

    return ret;
}

bool motor_control::write_duty2HW(float left_duty, float right_duty)
{
    uint8_t speed;
    if(left_duty == 0) {
        motor[MOTOR_FRONT_LEFT]->run(RELEASE);
        motor[MOTOR_REAR_LEFT]->run(RELEASE);
    }
    else if(left_duty > 0) {
        speed = (uint8_t)(left_duty*255);
        motor[MOTOR_FRONT_LEFT]->setSpeed(speed);
        motor[MOTOR_REAR_LEFT]->setSpeed(speed);
        motor[MOTOR_FRONT_LEFT]->run(FORWARD);
        motor[MOTOR_REAR_LEFT]->run(FORWARD);
    }
    else {
        speed = (uint8_t)(-1*left_duty*255);
        motor[MOTOR_FRONT_LEFT]->setSpeed(speed);
        motor[MOTOR_REAR_LEFT]->setSpeed(speed);
        motor[MOTOR_FRONT_LEFT]->run(BACKWARD);
        motor[MOTOR_REAR_LEFT]->run(BACKWARD);
    }

    if(right_duty == 0) {
        motor[MOTOR_FRONT_RIGHT]->run(RELEASE);
        motor[MOTOR_REAR_RIGHT]->run(RELEASE);
    }
    else if(right_duty > 0) {
        speed = (uint8_t)(right_duty*255);
        motor[MOTOR_FRONT_RIGHT]->setSpeed(speed);
        motor[MOTOR_REAR_RIGHT]->setSpeed(speed);
        motor[MOTOR_FRONT_RIGHT]->run(FORWARD);
        motor[MOTOR_REAR_RIGHT]->run(FORWARD);
    }
    else {
        speed = (uint8_t)(-1*right_duty*255);
        motor[MOTOR_FRONT_RIGHT]->setSpeed(speed);
        motor[MOTOR_REAR_RIGHT]->setSpeed(speed);
        motor[MOTOR_FRONT_RIGHT]->run(BACKWARD);
        motor[MOTOR_REAR_RIGHT]->run(BACKWARD);
    }

    return true;
}

