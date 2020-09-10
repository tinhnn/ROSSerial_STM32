/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "motor_driver.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

motor_driver::motor_driver()
{
    _motor_drive = PCA9685(0x60);
}

void motor_driver::init(void)
{
    _motor_drive.init(1000, &hi2c1);
    // Get motor pin control
    motor[MOTOR_FRONT_LEFT] = _motor_drive.getMotor(MOTOR_FRONT_LEFT + 1);
    motor[MOTOR_FRONT_RIGHT] = _motor_drive.getMotor(MOTOR_FRONT_RIGHT + 1);
    motor[MOTOR_REAR_LEFT] = _motor_drive.getMotor(MOTOR_REAR_LEFT + 1);
    motor[MOTOR_REAR_RIGHT] = _motor_drive.getMotor(MOTOR_REAR_RIGHT + 1);
}

bool motor_driver::writeVelocity(int64_t left_value, int64_t right_value)
{
    uint8_t duty;

    if(left_value == 0) {
        motor[MOTOR_FRONT_LEFT]->run(RELEASE);
        motor[MOTOR_REAR_LEFT]->run(RELEASE);
    }
    else if(left_value > 0) {
        duty = (uint8_t)(left_value*255/AUTOBOT_MAX_VELOCITY);
        motor[MOTOR_FRONT_LEFT]->setSpeed(duty);
        motor[MOTOR_REAR_LEFT]->setSpeed(duty);
        motor[MOTOR_FRONT_LEFT]->run(FORWARD);
        motor[MOTOR_REAR_LEFT]->run(FORWARD);
    }
    else {
        duty = (uint8_t)(-1*left_value*255/AUTOBOT_MAX_VELOCITY);
        motor[MOTOR_FRONT_LEFT]->setSpeed(duty);
        motor[MOTOR_REAR_LEFT]->setSpeed(duty);
        motor[MOTOR_FRONT_LEFT]->run(BACKWARD);
        motor[MOTOR_REAR_LEFT]->run(BACKWARD);
    }

    if(right_value == 0) {
        motor[MOTOR_FRONT_LEFT]->run(RELEASE);
        motor[MOTOR_REAR_LEFT]->run(RELEASE);
    }
    else if(right_value > 0) {
        duty = (uint8_t)(right_value*255/AUTOBOT_MAX_VELOCITY);
        motor[MOTOR_FRONT_LEFT]->setSpeed(duty);
        motor[MOTOR_REAR_LEFT]->setSpeed(duty);
        motor[MOTOR_FRONT_LEFT]->run(FORWARD);
        motor[MOTOR_REAR_LEFT]->run(FORWARD);
    }
    else {
        duty = (uint8_t)(-1*right_value*255/AUTOBOT_MAX_VELOCITY);
        motor[MOTOR_FRONT_LEFT]->setSpeed(duty);
        motor[MOTOR_REAR_LEFT]->setSpeed(duty);
        motor[MOTOR_FRONT_LEFT]->run(BACKWARD);
        motor[MOTOR_REAR_LEFT]->run(BACKWARD);
    }


    return true;
}

bool motor_driver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
    bool ret = false;
    float wheel_velocity_cmd[2];
    float lin_vel = value[LEFT];
    float ang_vel = value[RIGHT];

    wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -1*AUTOBOT_MAX_VELOCITY, AUTOBOT_MAX_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -1*AUTOBOT_MAX_VELOCITY, AUTOBOT_MAX_VELOCITY);

    ret = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);

    return ret;
}
