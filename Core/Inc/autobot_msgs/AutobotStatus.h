#ifndef _ROS_autobot_msgs_AutobotStatus_h
#define _ROS_autobot_msgs_AutobotStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace autobot_msgs
{

  class AutobotStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _uptime_type;
      _uptime_type uptime;
      typedef float _ros_control_loop_freq_type;
      _ros_control_loop_freq_type ros_control_loop_freq;
      typedef float _mcu_and_user_port_current_type;
      _mcu_and_user_port_current_type mcu_and_user_port_current;
      typedef float _left_driver_current_type;
      _left_driver_current_type left_driver_current;
      typedef float _right_driver_current_type;
      _right_driver_current_type right_driver_current;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef float _left_driver_voltage_type;
      _left_driver_voltage_type left_driver_voltage;
      typedef float _right_driver_voltage_type;
      _right_driver_voltage_type right_driver_voltage;
      typedef float _left_driver_temp_type;
      _left_driver_temp_type left_driver_temp;
      typedef float _right_driver_temp_type;
      _right_driver_temp_type right_driver_temp;
      typedef float _left_motor_temp_type;
      _left_motor_temp_type left_motor_temp;
      typedef float _right_motor_temp_type;
      _right_motor_temp_type right_motor_temp;
      typedef uint16_t _capacity_estimate_type;
      _capacity_estimate_type capacity_estimate;
      typedef float _charge_estimate_type;
      _charge_estimate_type charge_estimate;
      typedef bool _timeout_type;
      _timeout_type timeout;
      typedef bool _lockout_type;
      _lockout_type lockout;
      typedef bool _e_stop_type;
      _e_stop_type e_stop;
      typedef bool _ros_pause_type;
      _ros_pause_type ros_pause;
      typedef bool _no_battery_type;
      _no_battery_type no_battery;
      typedef bool _current_limit_type;
      _current_limit_type current_limit;

    AutobotStatus():
      header(),
      uptime(0),
      ros_control_loop_freq(0),
      mcu_and_user_port_current(0),
      left_driver_current(0),
      right_driver_current(0),
      battery_voltage(0),
      left_driver_voltage(0),
      right_driver_voltage(0),
      left_driver_temp(0),
      right_driver_temp(0),
      left_motor_temp(0),
      right_motor_temp(0),
      capacity_estimate(0),
      charge_estimate(0),
      timeout(0),
      lockout(0),
      e_stop(0),
      ros_pause(0),
      no_battery(0),
      current_limit(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->uptime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uptime >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uptime >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uptime >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uptime);
      offset += serializeAvrFloat64(outbuffer + offset, this->ros_control_loop_freq);
      offset += serializeAvrFloat64(outbuffer + offset, this->mcu_and_user_port_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_driver_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_driver_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->battery_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_driver_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_driver_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_driver_temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_driver_temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_motor_temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_motor_temp);
      *(outbuffer + offset + 0) = (this->capacity_estimate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->capacity_estimate >> (8 * 1)) & 0xFF;
      offset += sizeof(this->capacity_estimate);
      offset += serializeAvrFloat64(outbuffer + offset, this->charge_estimate);
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timeout);
      union {
        bool real;
        uint8_t base;
      } u_lockout;
      u_lockout.real = this->lockout;
      *(outbuffer + offset + 0) = (u_lockout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lockout);
      union {
        bool real;
        uint8_t base;
      } u_e_stop;
      u_e_stop.real = this->e_stop;
      *(outbuffer + offset + 0) = (u_e_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->e_stop);
      union {
        bool real;
        uint8_t base;
      } u_ros_pause;
      u_ros_pause.real = this->ros_pause;
      *(outbuffer + offset + 0) = (u_ros_pause.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_pause);
      union {
        bool real;
        uint8_t base;
      } u_no_battery;
      u_no_battery.real = this->no_battery;
      *(outbuffer + offset + 0) = (u_no_battery.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->no_battery);
      union {
        bool real;
        uint8_t base;
      } u_current_limit;
      u_current_limit.real = this->current_limit;
      *(outbuffer + offset + 0) = (u_current_limit.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_limit);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->uptime =  ((uint32_t) (*(inbuffer + offset)));
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->uptime);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ros_control_loop_freq));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mcu_and_user_port_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_driver_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_driver_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_driver_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_driver_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_driver_temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_driver_temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_motor_temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_motor_temp));
      this->capacity_estimate =  ((uint16_t) (*(inbuffer + offset)));
      this->capacity_estimate |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->capacity_estimate);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->charge_estimate));
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
      union {
        bool real;
        uint8_t base;
      } u_lockout;
      u_lockout.base = 0;
      u_lockout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lockout = u_lockout.real;
      offset += sizeof(this->lockout);
      union {
        bool real;
        uint8_t base;
      } u_e_stop;
      u_e_stop.base = 0;
      u_e_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->e_stop = u_e_stop.real;
      offset += sizeof(this->e_stop);
      union {
        bool real;
        uint8_t base;
      } u_ros_pause;
      u_ros_pause.base = 0;
      u_ros_pause.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_pause = u_ros_pause.real;
      offset += sizeof(this->ros_pause);
      union {
        bool real;
        uint8_t base;
      } u_no_battery;
      u_no_battery.base = 0;
      u_no_battery.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->no_battery = u_no_battery.real;
      offset += sizeof(this->no_battery);
      union {
        bool real;
        uint8_t base;
      } u_current_limit;
      u_current_limit.base = 0;
      u_current_limit.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_limit = u_current_limit.real;
      offset += sizeof(this->current_limit);
     return offset;
    }

    virtual const char * getType() override { return "autobot_msgs/AutobotStatus"; };
    virtual const char * getMD5() override { return "fd724379c53d89ec4629be3b235dc10d"; };

  };

}
#endif
