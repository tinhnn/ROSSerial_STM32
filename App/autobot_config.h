/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef AUTOBOT_CONFIG_H
#define AUTOBOT_CONFIG_H

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "ringbuffer.h"


/*******************************************************************************
 *  Callback Function prototype
 ******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);



/*******************************************************************************
 *  SUBSCRIBER TOPIC
 ******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
 *  PUBLISHER TOPIC
 ******************************************************************************/
/* Odometry */
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);


#endif /* AUTOBOT_CONFIG_H */
