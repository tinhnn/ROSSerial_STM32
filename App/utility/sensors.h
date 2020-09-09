/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>


#define USING_MPU9250
#if defined(USING_MPU9250)
#include "MPU9250.h"
#elif defined(USING_ICM_20648)
#include "ICM_20648.h"
#endif

class sensors
{
    private:
        MPU9250 _imu;
    public:
        sensors();
        void init(void);
        void calibrationGyro();
        void updateIMU(void);
        float* getOrientation(void);
};

#endif /* SENSORS_H */
