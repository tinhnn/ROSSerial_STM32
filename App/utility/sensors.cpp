/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "sensors.h"

sensors::sensors()
{
    // TODO
}

void sensors::init(void)
{
    // TODO
}

void sensors::calibrationGyro()
{
    // TODO
}

void sensors::updateIMU(void)
{
    // TODO
}

float* sensors::getOrientation(void)
{
    static float orientation[4];

    orientation[0] = _imu.quat[0];
    orientation[1] = _imu.quat[1];
    orientation[2] = _imu.quat[2];
    orientation[3] = _imu.quat[3];

    return orientation;
}