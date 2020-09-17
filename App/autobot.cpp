/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "autobot.h"
#include "autobot_config.h"

#include "main.h"

#define get_currenttime HAL_GetTick

/* ROS node handle */
ros::NodeHandle nh;

unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];
/* Calculation for odometry */
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM]   = {0, 0};
double  last_rad[WHEEL_NUM]         = {0.0, 0.0};
double  last_velocity[WHEEL_NUM]    = {0.0, 0.0};
float zero_velocity[WHEEL_NUM]      = {0.0, 0.0};
float goal_velocity[WHEEL_NUM]      = {0.0, 0.0};

/* software timer of Autobot */
static uint32_t evtTimer[MAX_EVT_TIMER];

/* ROS Parameter */
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];
char joint_state_header_frame_id[30];


/*******************************************************************************
 * GLOBAL FUNCTION
 ******************************************************************************/
void setup()
{
    // Initialize ROS node handle and pub/sub topic
    nh.initNode();

    nh.subscribe(cmd_vel_sub);
    nh.subscribe(reset_sub);

    nh.advertise(odom_pub);
    nh.advertise(joint_states_pub);

    tf_broadcaster.init(nh);

    // Initialize motor control board
    motor_control.init();

    // Initialize IMU
    sensor.init();

    // Init Odom
    initOdom();
    initJointStates();

    prev_update_time = get_currenttime();

}


void loop()
{
    uint32_t cur_t = get_currenttime();

    /*********** update velocity **********************************************/ 
    if((cur_t - evtTimer[UPDATE_MOTOR_VEL]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY)){
        if ((cur_t-evtTimer[REV_CMD_VEL]) > CONTROL_MOTOR_TIMEOUT)
        {
        	motor_control.write_velocity(zero_velocity[0], zero_velocity[1]);
        } 
        else {
        	motor_control.write_velocity(goal_velocity[0],goal_velocity[1]);
        }

        evtTimer[UPDATE_MOTOR_VEL] = cur_t;
    }

    /*********** publish topic ************************************************/
    if((cur_t - evtTimer[UPDATE_DRIVE_INFO]) >= (1000 / DRIVE_INFO_PUBLISH_FREQUENCY)){
        publishDriveInformation();
        evtTimer[UPDATE_DRIVE_INFO] = cur_t;
    }

    // Update the IMU unit
    sensor.updateIMU();

    // Start Gyro Calibration after ROS connection
    updateGyroCali(nh.connected());

    nh.spinOnce();
}


/*******************************************************************************
 * Private FUNCTION
 ******************************************************************************/
void initJointStates(void)
{
    static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

    joint_states.header.frame_id = joint_state_header_frame_id;
    joint_states.name            = joint_states_name;

    joint_states.name_length     = WHEEL_NUM;
    joint_states.position_length = WHEEL_NUM;
    joint_states.velocity_length = WHEEL_NUM;
    joint_states.effort_length   = WHEEL_NUM;
}

void initOdom(void)
{
    init_encoder = true;

    for (int index = 0; index < 3; index++)
    {
        odom_pose[index] = 0.0;
        odom_vel[index]  = 0.0;
    }

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;

    odom.twist.twist.linear.x  = 0.0;
    odom.twist.twist.angular.z = 0.0;
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

    orientation = sensor.getOrientation();
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

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void updateJointStates(void)
{
    static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
    static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
    //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

    joint_states_pos[LEFT]  = last_rad[LEFT];
    joint_states_pos[RIGHT] = last_rad[RIGHT];

    joint_states_vel[LEFT]  = last_velocity[LEFT];
    joint_states_vel[RIGHT] = last_velocity[RIGHT];

    joint_states.position = joint_states_pos;
    joint_states.velocity = joint_states_vel;
}

void updateGyroCali(bool isConnected)
{
    static bool isEnded = false;
    char log_msg[50];

    (void)(isConnected);

    if (nh.connected()) {
        if (isEnded == false)
        {
            sprintf(log_msg, "Start Calibration of Gyro");
            nh.loginfo(log_msg);

            sensor.calibrationGyro();

            sprintf(log_msg, "Calibration End");
            nh.loginfo(log_msg);

            isEnded = true;
        }
    }
    else {
        isEnded = false;
    }
}

/*******************************************************************************
 * Subscribe topic callback function
 ******************************************************************************/

/*******************************************************************************
 * FUNC: commandVelocityCallback
 * Description: cmd_vel callback function
 * Param:
 ******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    goal_velocity[LINEAR]  = cmd_vel_msg.linear.x;
    goal_velocity[ANGULAR] = cmd_vel_msg.angular.z;

    goal_velocity[LINEAR]  = constrain(goal_velocity[LINEAR],  MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    goal_velocity[ANGULAR] = constrain(goal_velocity[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    evtTimer[REV_CMD_VEL] = get_currenttime();
}

/*******************************************************************************
 * FUNC: resetCallback
 * Description: reset callback function
 * Param:
 ******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{
    char log_msg[50];

    (void)(reset_msg);

    sprintf(log_msg, "Start Calibration of Gyro");
    nh.loginfo(log_msg);

    sensor.calibrationGyro();

    sprintf(log_msg, "Calibration End");
    nh.loginfo(log_msg);

    initOdom();

    sprintf(log_msg, "Reset Odometry");
    nh.loginfo(log_msg);  
}

/*******************************************************************************
* Publish topic function
*******************************************************************************/
void publishDriveInformation(void)
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

    // publish tf
    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    // publish joint states
    updateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);
}

/********************************** END FILE **********************************/
