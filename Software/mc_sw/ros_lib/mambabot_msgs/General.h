#ifndef _ROS_mambabot_msgs_General_h
#define _ROS_mambabot_msgs_General_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mambabot_msgs
{

  class General : public ros::Msg
  {
    public:
      typedef int32_t _robot_base_voltage_type;
      _robot_base_voltage_type robot_base_voltage;
      typedef int32_t _motor1_current_type;
      _motor1_current_type motor1_current;
      typedef int32_t _motor2_current_type;
      _motor2_current_type motor2_current;
      typedef int32_t _motor3_current_type;
      _motor3_current_type motor3_current;
      typedef int32_t _motor4_current_type;
      _motor4_current_type motor4_current;
      typedef int32_t _inside_temperature_type;
      _inside_temperature_type inside_temperature;

    General():
      robot_base_voltage(0),
      motor1_current(0),
      motor2_current(0),
      motor3_current(0),
      motor4_current(0),
      inside_temperature(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_robot_base_voltage;
      u_robot_base_voltage.real = this->robot_base_voltage;
      *(outbuffer + offset + 0) = (u_robot_base_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_base_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_base_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_base_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robot_base_voltage);
      union {
        int32_t real;
        uint32_t base;
      } u_motor1_current;
      u_motor1_current.real = this->motor1_current;
      *(outbuffer + offset + 0) = (u_motor1_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor1_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor1_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor1_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor1_current);
      union {
        int32_t real;
        uint32_t base;
      } u_motor2_current;
      u_motor2_current.real = this->motor2_current;
      *(outbuffer + offset + 0) = (u_motor2_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor2_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor2_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor2_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor2_current);
      union {
        int32_t real;
        uint32_t base;
      } u_motor3_current;
      u_motor3_current.real = this->motor3_current;
      *(outbuffer + offset + 0) = (u_motor3_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor3_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor3_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor3_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor3_current);
      union {
        int32_t real;
        uint32_t base;
      } u_motor4_current;
      u_motor4_current.real = this->motor4_current;
      *(outbuffer + offset + 0) = (u_motor4_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor4_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor4_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor4_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor4_current);
      union {
        int32_t real;
        uint32_t base;
      } u_inside_temperature;
      u_inside_temperature.real = this->inside_temperature;
      *(outbuffer + offset + 0) = (u_inside_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inside_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_inside_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_inside_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->inside_temperature);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_robot_base_voltage;
      u_robot_base_voltage.base = 0;
      u_robot_base_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_base_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_base_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_base_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->robot_base_voltage = u_robot_base_voltage.real;
      offset += sizeof(this->robot_base_voltage);
      union {
        int32_t real;
        uint32_t base;
      } u_motor1_current;
      u_motor1_current.base = 0;
      u_motor1_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor1_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor1_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor1_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor1_current = u_motor1_current.real;
      offset += sizeof(this->motor1_current);
      union {
        int32_t real;
        uint32_t base;
      } u_motor2_current;
      u_motor2_current.base = 0;
      u_motor2_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor2_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor2_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor2_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor2_current = u_motor2_current.real;
      offset += sizeof(this->motor2_current);
      union {
        int32_t real;
        uint32_t base;
      } u_motor3_current;
      u_motor3_current.base = 0;
      u_motor3_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor3_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor3_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor3_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor3_current = u_motor3_current.real;
      offset += sizeof(this->motor3_current);
      union {
        int32_t real;
        uint32_t base;
      } u_motor4_current;
      u_motor4_current.base = 0;
      u_motor4_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor4_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor4_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor4_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor4_current = u_motor4_current.real;
      offset += sizeof(this->motor4_current);
      union {
        int32_t real;
        uint32_t base;
      } u_inside_temperature;
      u_inside_temperature.base = 0;
      u_inside_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inside_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_inside_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_inside_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->inside_temperature = u_inside_temperature.real;
      offset += sizeof(this->inside_temperature);
     return offset;
    }

    const char * getType(){ return "mambabot_msgs/General"; };
    const char * getMD5(){ return "b70100424bc96f7e436c41ab0e9792d8"; };

  };

}
#endif