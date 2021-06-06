#ifndef _ROS_mambabot_msgs_Diagnostic_h
#define _ROS_mambabot_msgs_Diagnostic_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mambabot_msgs
{

  class Diagnostic : public ros::Msg
  {
    public:
      typedef const char* _voltage_error_type;
      _voltage_error_type voltage_error;
      typedef const char* _temperature_error_type;
      _temperature_error_type temperature_error;
      typedef const char* _motor_current_error_type;
      _motor_current_error_type motor_current_error;
      typedef const char* _det_error_type;
      _det_error_type det_error;

    Diagnostic():
      voltage_error(""),
      temperature_error(""),
      motor_current_error(""),
      det_error("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_voltage_error = strlen(this->voltage_error);
      varToArr(outbuffer + offset, length_voltage_error);
      offset += 4;
      memcpy(outbuffer + offset, this->voltage_error, length_voltage_error);
      offset += length_voltage_error;
      uint32_t length_temperature_error = strlen(this->temperature_error);
      varToArr(outbuffer + offset, length_temperature_error);
      offset += 4;
      memcpy(outbuffer + offset, this->temperature_error, length_temperature_error);
      offset += length_temperature_error;
      uint32_t length_motor_current_error = strlen(this->motor_current_error);
      varToArr(outbuffer + offset, length_motor_current_error);
      offset += 4;
      memcpy(outbuffer + offset, this->motor_current_error, length_motor_current_error);
      offset += length_motor_current_error;
      uint32_t length_det_error = strlen(this->det_error);
      varToArr(outbuffer + offset, length_det_error);
      offset += 4;
      memcpy(outbuffer + offset, this->det_error, length_det_error);
      offset += length_det_error;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_voltage_error;
      arrToVar(length_voltage_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_voltage_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_voltage_error-1]=0;
      this->voltage_error = (char *)(inbuffer + offset-1);
      offset += length_voltage_error;
      uint32_t length_temperature_error;
      arrToVar(length_temperature_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_temperature_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_temperature_error-1]=0;
      this->temperature_error = (char *)(inbuffer + offset-1);
      offset += length_temperature_error;
      uint32_t length_motor_current_error;
      arrToVar(length_motor_current_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor_current_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor_current_error-1]=0;
      this->motor_current_error = (char *)(inbuffer + offset-1);
      offset += length_motor_current_error;
      uint32_t length_det_error;
      arrToVar(length_det_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_det_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_det_error-1]=0;
      this->det_error = (char *)(inbuffer + offset-1);
      offset += length_det_error;
     return offset;
    }

    const char * getType(){ return "mambabot_msgs/Diagnostic"; };
    const char * getMD5(){ return "85b47aed5e47a831f8efd717e75f10d6"; };

  };

}
#endif