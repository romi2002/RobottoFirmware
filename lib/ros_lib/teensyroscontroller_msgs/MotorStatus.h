#ifndef _ROS_teensyroscontroller_msgs_MotorStatus_h
#define _ROS_teensyroscontroller_msgs_MotorStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace teensyroscontroller_msgs
{

  class MotorStatus : public ros::Msg
  {
    public:
      typedef int32_t _currentPose_type;
      _currentPose_type currentPose;
      typedef float _currentTest_type;
      _currentTest_type currentTest;

    MotorStatus():
      currentPose(0),
      currentTest(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_currentPose;
      u_currentPose.real = this->currentPose;
      *(outbuffer + offset + 0) = (u_currentPose.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentPose.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentPose.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentPose.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentPose);
      offset += serializeAvrFloat64(outbuffer + offset, this->currentTest);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_currentPose;
      u_currentPose.base = 0;
      u_currentPose.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentPose.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentPose.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentPose.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentPose = u_currentPose.real;
      offset += sizeof(this->currentPose);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->currentTest));
     return offset;
    }

    const char * getType(){ return "teensyroscontroller_msgs/MotorStatus"; };
    const char * getMD5(){ return "9cd463332153d6d4c2d395df0558ca25"; };

  };

}
#endif
