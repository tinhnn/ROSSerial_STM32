/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "DFR0592.h"



DFR0592::DFR0592(/* args */)
{
}

DFR0592::~DFR0592()
{
}

bool DFR0592::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
    return 0;
}

float* DFR0592::getOrientation(void)
{
    static float orientation[4];

    //orientation[0] = imu_.quat[0];
    //orientation[1] = imu_.quat[1];
    //orientation[2] = imu_.quat[2];
    //orientation[3] = imu_.quat[3];

    return orientation;
}
