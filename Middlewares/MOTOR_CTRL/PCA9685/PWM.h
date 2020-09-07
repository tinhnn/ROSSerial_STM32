/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "main.h"

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

class PWM {
public:
    PWM(uint8_t addr = 0x40);
    void begin(I2C_HandleTypeDef *i2c_hdl);
    void reset(I2C_HandleTypeDef *i2c_hdl);
    void setPWMFreq(I2C_HandleTypeDef *i2c_hdl, float freq);
    void setPWM(I2C_HandleTypeDef *i2c_hdl, uint8_t num, uint16_t on, uint16_t off);

private:
    uint8_t _i2caddr;

    uint8_t read8(I2C_HandleTypeDef *i2c_hdl, uint8_t addr);
    void write8(I2C_HandleTypeDef *i2c_hdl, uint8_t addr, uint8_t d);
};

#endif /* PWM */
