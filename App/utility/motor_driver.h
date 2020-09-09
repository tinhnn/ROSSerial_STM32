/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

class motor_driver
{
    private:
        uint32_t _left_wheel_id;
    public:
        motor_driver();
        void init(void);
        bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

};


#endif /* MOTOR_DRIVER_H */
