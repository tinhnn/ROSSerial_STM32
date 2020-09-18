/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef PCA9685_H
#define PCA9685_H

#include "main.h"
#include "PWM.h"

#define MOTOR1_A    2
#define MOTOR1_B    3
#define MOTOR2_A    1
#define MOTOR2_B    4
#define MOTOR4_A    0
#define MOTOR4_B    6
#define MOTOR3_A    5
#define MOTOR3_B    7

#define FORWARD     1
#define BACKWARD    2
#define BRAKE       3
#define RELEASE     4

#define SINGLE      1
#define DOUBLE      2
#define INTERLEAVE  3
#define MICROSTEP   4

#define LOW         0
#define HIGH        1

class PCA9685;

/********* Object that controls and keeps state for a single DC motor *********/
class DCMotor {
public:
    DCMotor(void);
    friend class PCA9685;         /* Create DC motors */
    void run(uint8_t);
    void setSpeed(uint8_t);

private:
    uint8_t PWMpin, IN1pin, IN2pin;
    PCA9685 *MC;
    uint8_t motornum;
};

/****** Object that controls and keeps state for a single stepper motor *******/
class StepperMotor {
public:
    StepperMotor(void);
    void setSpeed(uint16_t);

    void step(uint16_t steps, uint8_t dir, uint8_t style = SINGLE);
    uint8_t onestep(uint8_t dir, uint8_t style);
    void release(void);

    friend class PCA9685;         /* Create stepper motors */

private:
    uint32_t usperstep;

    uint8_t PWMApin, AIN1pin, AIN2pin;
    uint8_t PWMBpin, BIN1pin, BIN2pin;
    uint16_t revsteps;          /* steps per revolution */
    uint8_t currentstep;
    PCA9685 *MC;
    uint8_t steppernum;
};

/** Object that controls and keeps state for the whole motor shield.
    Use it to create DC and Stepper motor objects! */
class PCA9685
{
private:
    I2C_HandleTypeDef *_i2c_hdl;
    uint8_t _addr;
    uint16_t _freq;
    DCMotor _dcmotors[4];
    StepperMotor _steppers[2];
    PWM _pwm;


public:
    PCA9685(uint8_t addr = 0x60);
    
    void init(uint16_t freq = 1600);
    DCMotor *getMotor(uint8_t n);
    StepperMotor *getStepper(uint16_t steps, uint8_t n);
    friend class DCMotor;
    void setPin(uint8_t pin, bool val);
    void setPWM(uint8_t pin, uint16_t val);
};



#endif /* PCA9685_H */
