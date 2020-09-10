/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include <stdio.h>
#include "PCA9685.h"


/*******************************************************************************
 * PCA9685 motor HAT function
 ******************************************************************************/
PCA9685::PCA9685(uint8_t addr)
{
    _addr = addr;
    _pwm = PWM(_addr);
}


void PCA9685::init(uint16_t freq, I2C_HandleTypeDef *i2c_hdl)
{
    _i2c_hdl = i2c_hdl;
    _pwm.begin(_i2c_hdl);
    _freq = freq;
    _pwm.setPWMFreq(_i2c_hdl, _freq); // This is the maximum PWM frequency
    for (uint8_t i = 0; i < 16; i++)
        _pwm.setPWM(_i2c_hdl, i, 0, 0);
}

void PCA9685::setPWM(uint8_t pin, uint16_t value) {
    if (value > 4095) {
        _pwm.setPWM(_i2c_hdl, pin, 4096, 0);
    } else {
        _pwm.setPWM(_i2c_hdl, pin, 0, value);
    }
}

void PCA9685::setPin(uint8_t pin, bool value) {
    if (value == LOW) {
        _pwm.setPWM(_i2c_hdl, pin, 0, 0);
    } else {
        _pwm.setPWM(_i2c_hdl, pin, 4096, 0);
    }
}

DCMotor *PCA9685::getMotor(uint8_t num) {
    if (num > 4)
        return NULL;

    num--;

    if (_dcmotors[num].motornum == 0) {
        _dcmotors[num].motornum = num;
        _dcmotors[num].MC = this;
        uint8_t pwm, in1, in2;
        if (num == 0) {             /* DC Motor 1 */
            pwm = 8;
            in2 = 9;
            in1 = 10;
        } else if (num == 1) {      /* DC Motor 2 */
            pwm = 13;
            in2 = 12;
            in1 = 11;
        } else if (num == 2) {      /* DC Motor 3 */
            pwm = 2;
            in2 = 3;
            in1 = 4;
        } else {                    /* DC Motor 4 */
            pwm = 7;
            in2 = 6;
            in1 = 5;
        }
        _dcmotors[num].PWMpin = pwm;
        _dcmotors[num].IN1pin = in1;
        _dcmotors[num].IN2pin = in2;
    }

    return &_dcmotors[num];
}

StepperMotor *PCA9685::getStepper(uint16_t steps, uint8_t num) {
    if (num > 2)
        return NULL;

    num--;

    if (_steppers[num].steppernum == 0) {
    	_steppers[num].steppernum = num;
    	_steppers[num].revsteps = steps;
    	_steppers[num].MC = this;
        uint8_t pwma, pwmb, ain1, ain2, bin1, bin2;
        if (num == 0) {             /* Stepper Motor 1 */
            pwma = 8;
            ain2 = 9;
            ain1 = 10;
            pwmb = 13;
            bin2 = 12;
            bin1 = 11;
        } else {                    /* Stepper Motor 2 */
            pwma = 2;
            ain2 = 3;
            ain1 = 4;
            pwmb = 7;
            bin2 = 6;
            bin1 = 5;
        }
        _steppers[num].PWMApin = pwma;
        _steppers[num].PWMBpin = pwmb;
        _steppers[num].AIN1pin = ain1;
        _steppers[num].AIN2pin = ain2;
        _steppers[num].BIN1pin = bin1;
        _steppers[num].BIN2pin = bin2;
    }
    return &_steppers[num];
}


/*******************************************************************************
 * Utility function
 ******************************************************************************/

/************* DC motors *************/
DCMotor::DCMotor()
{
    MC = NULL;
    motornum = 0;
    PWMpin = IN1pin = IN2pin = 0;
}

void DCMotor::run(uint8_t cmd)
{
    switch (cmd) {
        case FORWARD:
            MC->setPin(IN2pin, LOW);
            MC->setPin(IN1pin, HIGH);
            break;
        case BACKWARD:
            MC->setPin(IN1pin, LOW);
            MC->setPin(IN2pin, HIGH);
            break;
        case RELEASE:
            MC->setPin(IN1pin, LOW);
            MC->setPin(IN2pin, LOW);
            break;
    }
}

void DCMotor::setSpeed(uint8_t speed)
{
    MC->setPWM(PWMpin, speed * 16);
}

/************* Stepper motors *************/
StepperMotor::StepperMotor()
{
    // TODO
}

void StepperMotor::setSpeed(uint16_t rpm)
{
    // TODO
}

void StepperMotor::release(void)
{
    // TODO
}

void StepperMotor::step(uint16_t steps, uint8_t dir, uint8_t style)
{
    // TODO
}

uint8_t StepperMotor::onestep(uint8_t dir, uint8_t style)
{
    // TODO
    return 0;
}
