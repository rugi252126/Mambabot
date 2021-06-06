#ifndef _ROS_mambabot_msgs_ProjectInfo_h
#define _ROS_mambabot_msgs_ProjectInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mambabot_msgs
{

  class ProjectInfo : public ros::Msg
  {
    public:
      typedef const char* _author_type;
      _author_type author;
      typedef const char* _project_status_type;
      _project_status_type project_status;

    ProjectInfo():
      author(""),
      project_status("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_author = strlen(this->author);
      varToArr(outbuffer + offset, length_author);
      offset += 4;
      memcpy(outbuffer + offset, this->author, length_author);
      offset += length_author;
      uint32_t length_project_status = strlen(this->project_status);
      varToArr(outbuffer + offset, length_project_status);
      offset += 4;
      memcpy(outbuffer + offset, this->project_status, length_project_status);
      offset += length_project_status;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_author;
      arrToVar(length_author, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_author; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_author-1]=0;
      this->author = (char *)(inbuffer + offset-1);
      offset += length_author;
      uint32_t length_project_status;
      arrToVar(length_project_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_project_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_project_status-1]=0;
      this->project_status = (char *)(inbuffer + offset-1);
      offset += length_project_status;
     return offset;
    }

    const char * getType(){ return "mambabot_msgs/ProjectInfo"; };
    const char * getMD5(){ return "920e8c4a7d15f9fb51b61d7f48866c01"; };

  };

}
#endif