/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include <stdio.h>
#include "DFR0592.h"


DFR0592::DFR0592(/* args */)
{
}

DFR0592::~DFR0592()
{
}

void DFR0592::init(void)
{
    ;
}

bool DFR0592::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
	//float vel_lin_x = value[0];
	//float vel_ang_z = value[1];
	printf("set velocity\r\n");
    return 0;
}

float* DFR0592::getOrientation(void)
{
    static float orientation[4];

    //orientation[0] = imu_.quat[0];
    //orientation[1] = imu_.quat[1];
    //orientation[2] = imu_.quat[2];
    //orientation[3] = imu_.quat[3];
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
    orientation[3] = 0;


    return orientation;
}
