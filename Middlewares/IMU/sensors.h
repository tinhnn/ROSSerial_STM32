/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

/* Select IMU sensor */
#if defined(USING_MPU9250)      /* MPU9250 is end of life, replace by ICM20648and ICM20948 */
#include "MPU9250/MPU9250.h"
#elif defined(USING_ICM_20648)
#include "ICM/ICM_20648.h"
#elif defined(USING_ICM_20948)
#include "ICM/ICM_20948.h"
#endif

class sensors
{
public:
    sensors();
    void init(void);
    void calibrationGyro();
    void updateIMU(void);
    float* getOrientation(void);

private:
#if defined(USING_MPU9250)
    MPU9250 _imu;
#elif defined(USING_ICM_20648)
    ICM20648 _imu;
#elif defined(USING_ICM_20948)
    ICM20948 _imu;
#endif
    //
};

#endif /* SENSORS_H */
