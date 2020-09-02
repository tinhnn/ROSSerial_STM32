/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "autobot.h"
#include "autobot_config.h"




ros::NodeHandle nh;

void setup()
{
    nh.initNode();

    nh.subscribe(cmd_vel_sub);
    nh.advertise(odom_pub);
}


void loop()
{
    // publish topic
    nh.spinOnce();
    //
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    // TODO
}
