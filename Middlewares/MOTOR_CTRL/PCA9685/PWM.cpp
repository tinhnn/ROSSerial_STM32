/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "PWM.h"
#include "main.h"
#include <math.h>

PWM::PWM(uint8_t addr) {
    _i2caddr = addr<<1;
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
    uint32_t ret;
    uint8_t reg_addr;
    uint8_t message[4];

    reg_addr = LED0_ON_L + 4 * num;
    message[0] = on & 0xFF;
    message[1] = on >> 8;
    message[2] = off & 0xFF;
    message[3] = off >> 8;

    ret = HAL_I2C_Mem_Write(i2c_hdl, _i2caddr, reg_addr, 1, message, 4, 1000);
    if(!ret){
        // error handle
    }

}

uint8_t PWM::read8(I2C_HandleTypeDef *i2c_hdl, uint8_t addr) {
    uint32_t ret;
    uint8_t rx = 0;

    ret = HAL_I2C_Mem_Read(i2c_hdl, _i2caddr, addr, 1, &rx, 1, 1000);
    if(!ret){
        // error handle
    }
    
    return rx;
}

void PWM::write8(I2C_HandleTypeDef *i2c_hdl, uint8_t addr, uint8_t d) {
    uint32_t ret;

    ret = HAL_I2C_Mem_Write(i2c_hdl, _i2caddr, addr, 1, &d, 1, 1000);
    if(!ret){
        // error handle
    }
}
