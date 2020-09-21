/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef AUTOBOT_CONFIG_H
#define AUTOBOT_CONFIG_H

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "motor_controller.h"
#include "sensors.h"


/*******************************************************************************
 *  DEFINE
 ******************************************************************************/
#define CONTROL_MOTOR_SPEED_FREQUENCY               30      //hz
#define CONTROL_MOTOR_TIMEOUT                       500     //ms
#define DRIVE_INFO_PUBLISH_FREQUENCY                30      //hz

#define WHEEL_NUM                           2

#define LEFT                                0
#define RIGHT                               1

#define LINEAR                              0
#define ANGULAR                             1

#define TICK2RAD                            0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define abs(x)                  ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>=(high)?(high):(amt)))

typedef enum EVENT_TIMER_E{
	UPDATE_MOTOR_VEL,
    UPDATE_DRIVE_INFO,
    REV_CMD_VEL,
    MAX_EVT_TIMER,
}EVENT_TIMER;

/*******************************************************************************
 *  Variable
 ******************************************************************************/
/* Declaration for motor */
motor_control motor_control;
sensors sensor;


/*******************************************************************************
 *  Function prototype
 ******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

void publishDriveInformation(void);
void initOdom(void);
void initJointStates(void);
void updateGyroCali(bool isConnected);

/*******************************************************************************
 *  SUBSCRIBER TOPIC
 ******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
 *  PUBLISHER TOPIC
 ******************************************************************************/
/***************** Odometry *****************/
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint state
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/********** Transform Broadcaster **********/
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

#endif /* AUTOBOT_CONFIG_H */
