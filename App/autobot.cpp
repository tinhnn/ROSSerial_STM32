/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "autobot.h"
#include "autobot_config.h"

#include "main.h"

#define get_currenttime HAL_GetTick

ros::NodeHandle nh;

unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/********* PROTOTYPE FUNCTION *************************************************/
void publishOdometry(void);


/*******************************************************************************
 *
 ******************************************************************************/
void setup()
{
    // Initialize ROS node handle and pub/sub topic
    nh.initNode();

    nh.subscribe(cmd_vel_sub);
    nh.advertise(odom_pub);

    // Initialize motor board
    
}


void loop()
{
    uint32_t cur_t = get_currenttime();

    /*********** update velocity **********************************************/ 
    if((cur_t - evtTimer[UPDATE_MOTOR_VEL]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY)){
        if ((cur_t-evtTimer[REV_CMD_VEL]) > CONTROL_MOTOR_TIMEOUT)
        {
            motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity);
        } 
        else {
            motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
        }

        evtTimer[UPDATE_MOTOR_VEL] = cur_t;
    }

    /*********** publish topic ************************************************/
    if((cur_t - evtTimer[UPDATE_ODOM]) >= (1000 / ODOMETRY_PUBLISH_FREQUENCY)){
        publishOdometry();
        evtTimer[UPDATE_ODOM] = cur_t;
    }

    nh.spinOnce();
}

/*******************************************************************************
 *
 ******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    goal_velocity[LINEAR]  = cmd_vel_msg.linear.x;
    goal_velocity[ANGULAR] = cmd_vel_msg.angular.z;

    goal_velocity[LINEAR]  = constrain(goal_velocity[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    goal_velocity[ANGULAR] = constrain(goal_velocity[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    evtTimer[REV_CMD_VEL] = get_currenttime();
}

bool calcOdometry(double diff_time)
{
    float* orientation;
    double wheel_l, wheel_r;      // rotation value of wheel [rad]
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;
    double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
    double step_time;

    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = theta = 0.0;
    v = w = 0.0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
    return false;

    wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

    if (isnan(wheel_l))
    wheel_l = 0.0;

    if (isnan(wheel_r))
    wheel_r = 0.0;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;

    orientation = motor_driver.getOrientation();
    theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

    delta_theta = theta - last_theta;

    // compute odometric pose
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    // compute odometric instantaneouse velocity

    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT]  = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    last_theta = theta;

    return true;
}

void updateOdometry(void)
{
    odom.header.frame_id = odom_header_frame_id;
    odom.child_frame_id  = odom_child_frame_id;

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Publish msgs (odometry)
*******************************************************************************/
void publishOdometry(void)
{
	unsigned long time_now = get_currenttime();
	unsigned long step_time = time_now - prev_update_time;
	prev_update_time = time_now;
	ros::Time stamp_now = nh.now();

    // calculate odometry
    calcOdometry((double)(step_time * 0.001));

    // publish odometry
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);
}

