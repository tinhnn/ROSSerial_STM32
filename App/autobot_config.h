/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef AUTOBOT_CONFIG_H
#define AUTOBOT_CONFIG_H

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define USING_DFR0592

#ifdef USING_DFR0592        /* Using DFR0592 motor control board */
#include "board/DFR0592.h"
#else                       /* using nucleo motor control board*/
#include "board/nucleo_f746zg.h"
#endif


/*******************************************************************************
 *  DEFINE
 ******************************************************************************/
#define CONTROL_MOTOR_SPEED_FREQUENCY               30      //hz
#define CONTROL_MOTOR_TIMEOUT                       500     //ms
#define ODOMETRY_PUBLISH_FREQUENCY                  30      //hz

#define WHEEL_NUM                           2

#define LEFT                                0
#define RIGHT                               1

#define LINEAR                              0
#define ANGULAR                             1

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define abs(x)                  ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>=(high)?(high):(amt)))

typedef enum EVENT_TIMER_E{
	UPDATE_MOTOR_VEL,
    UPDATE_ODOM,
    REV_CMD_VEL,
    MAX_EVT_TIMER,
}EVENT_TIMER;

/*******************************************************************************
 *  Variable
 ******************************************************************************/
/* Declaration for motor */
#ifdef USING_DFR0592        /* Using DFR0592 motor control board */
DFR0592 motor_driver;
#else                       /* using nucleo motor control board*/
NUCLEO_F746ZG motor_driver;
#endif
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};

/* software timer of Autobot */
static uint32_t evtTimer[MAX_EVT_TIMER];

/************ ROS Parameter ***************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

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
