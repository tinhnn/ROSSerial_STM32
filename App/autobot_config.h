/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef AUTOBOT_CONFIG_H
#define AUTOBOT_CONFIG_H

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define USING_PCA9685

#if defined(USING_DFR0592)           /* Using DFR0592 motor control board */
#include "DFR0592/DFR0592.h"
#elif defined(USING_PCA9685)         /* Using PCA9685 motor control board */
#include "PCA9685/PCA9685.h"
#else                               /* using nucleo motor control board*/
#include "NUCLEO_F746ZG/nucleo_f746zg.h"
#endif


/*******************************************************************************
 *  DEFINE
 ******************************************************************************/
#define CONTROL_MOTOR_SPEED_FREQUENCY               30       //hz
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
#if defined(USING_DFR0592)          /* Using DFR0592 motor control board */
DFR0592 motor_driver;
#elif defined(USING_PCA9685)        /* Using PCA9685 motor control board */
PCA9685 motor_driver;
#else                               /* using nucleo motor control board*/
NUCLEO_F746ZG motor_driver;
#endif


/*******************************************************************************
 *  Function prototype
 ******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

void publishDriveInformation(void);
void initOdom(void);

/*******************************************************************************
 *  SUBSCRIBER TOPIC
 ******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
 *  PUBLISHER TOPIC
 ******************************************************************************/
/***************** Odometry *****************/
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

/********** Transform Broadcaster **********/
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

#endif /* AUTOBOT_CONFIG_H */
