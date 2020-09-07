/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "PWM.h"
#include "main.h"
#include <math.h>

PWM::PWM(uint8_t addr) {
    _i2caddr = addr;
}

void PWM::begin(I2C_HandleTypeDef *i2c_hdl) {
    reset(i2c_hdl);
}

void PWM::reset(I2C_HandleTypeDef *i2c_hdl) { write8(i2c_hdl, PCA9685_MODE1, 0x0); }

void PWM::setPWMFreq(I2C_HandleTypeDef *i2c_hdl, float freq) {

    freq *= 0.9;

    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval + 0.5);

    uint8_t oldmode = read8(i2c_hdl, PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    write8(i2c_hdl, PCA9685_MODE1, newmode);            // go to sleep
    write8(i2c_hdl, PCA9685_PRESCALE, prescale);        // set the prescaler
    write8(i2c_hdl, PCA9685_MODE1, oldmode);
    HAL_Delay(5);
    write8(i2c_hdl, PCA9685_MODE1, oldmode | 0xa1);
}

void PWM::setPWM(I2C_HandleTypeDef *i2c_hdl, uint8_t num, uint16_t on, uint16_t off) {
    uint8_t tx;
    tx = LED0_ON_L + 4 * num;
    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &tx, 1);
    tx = (uint8_t)on;
    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &tx, 1);
    tx = (uint8_t)(on >> 8);
    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &tx, 1);
    tx = (uint8_t)off;
    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &tx, 1);
    tx = (uint8_t)(off >> 8);
    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &tx, 1);

    while (HAL_I2C_GetState(i2c_hdl) != HAL_I2C_STATE_READY) {}
}

uint8_t PWM::read8(I2C_HandleTypeDef *i2c_hdl, uint8_t addr) {
    uint8_t rx = 0;

    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &addr, 1);
    while (HAL_I2C_GetState(i2c_hdl) != HAL_I2C_STATE_READY) {}

    HAL_I2C_Master_Receive_IT(i2c_hdl, _i2caddr, &rx, 1);
    return rx;
}

void PWM::write8(I2C_HandleTypeDef *i2c_hdl, uint8_t addr, uint8_t d) {
    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &addr, 1);
    HAL_I2C_Master_Transmit_IT(i2c_hdl, _i2caddr, &d, 1);
    while (HAL_I2C_GetState(i2c_hdl) != HAL_I2C_STATE_READY) {}
}
