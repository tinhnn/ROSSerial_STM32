/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "PCA9685/PCA9685.h"

#define MOTOR_FRONT_LEFT                0
#define MOTOR_FRONT_RIGHT               1
#define MOTOR_REAR_LEFT                 2
#define MOTOR_REAR_RIGHT                3
#define MOTOR_ID_MAX                    4

#define LEFT                            0
#define RIGHT                           1
#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309

#define AUTOBOT_MAX_VELOCITY            1           //

#define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>=(high)?(high):(amt)))

class motor_driver
{
    private:
        PCA9685 _motor_drive;
        DCMotor *motor[MOTOR_ID_MAX];
        uint8_t _left_wheel_id;
        uint8_t _right_wheel_id;

        bool writeVelocity(int64_t left_value, int64_t right_value);
    public:
        motor_driver();
        void init(void);
        bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

};


#endif /* MOTOR_DRIVER_H */
