#ifndef _ROS_mambabot_msgs_LogisticData_h
#define _ROS_mambabot_msgs_LogisticData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mambabot_msgs
{

  class LogisticData : public ros::Msg
  {
    public:
      typedef const char* _sw_version_type;
      _sw_version_type sw_version;
      typedef const char* _sw_internal_status_type;
      _sw_internal_status_type sw_internal_status;
      typedef const char* _hw_version_type;
      _hw_version_type hw_version;
      typedef const char* _variant_type;
      _variant_type variant;

    LogisticData():
      sw_version(""),
      sw_internal_status(""),
      hw_version(""),
      variant("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_sw_version = strlen(this->sw_version);
      varToArr(outbuffer + offset, length_sw_version);
      offset += 4;
      memcpy(outbuffer + offset, this->sw_version, length_sw_version);
      offset += length_sw_version;
      uint32_t length_sw_internal_status = strlen(this->sw_internal_status);
      varToArr(outbuffer + offset, length_sw_internal_status);
      offset += 4;
      memcpy(outbuffer + offset, this->sw_internal_status, length_sw_internal_status);
      offset += length_sw_internal_status;
      uint32_t length_hw_version = strlen(this->hw_version);
      varToArr(outbuffer + offset, length_hw_version);
      offset += 4;
      memcpy(outbuffer + offset, this->hw_version, length_hw_version);
      offset += length_hw_version;
      uint32_t length_variant = strlen(this->variant);
      varToArr(outbuffer + offset, length_variant);
      offset += 4;
      memcpy(outbuffer + offset, this->variant, length_variant);
      offset += length_variant;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_sw_version;
      arrToVar(length_sw_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sw_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sw_version-1]=0;
      this->sw_version = (char *)(inbuffer + offset-1);
      offset += length_sw_version;
      uint32_t length_sw_internal_status;
      arrToVar(length_sw_internal_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sw_internal_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sw_internal_status-1]=0;
      this->sw_internal_status = (char *)(inbuffer + offset-1);
      offset += length_sw_internal_status;
      uint32_t length_hw_version;
      arrToVar(length_hw_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hw_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hw_version-1]=0;
      this->hw_version = (char *)(inbuffer + offset-1);
      offset += length_hw_version;
      uint32_t length_variant;
      arrToVar(length_variant, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_variant; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_variant-1]=0;
      this->variant = (char *)(inbuffer + offset-1);
      offset += length_variant;
     return offset;
    }

    const char * getType(){ return "mambabot_msgs/LogisticData"; };
    const char * getMD5(){ return "28162334e91169ace135285b854db196"; };

  };

}
#endif