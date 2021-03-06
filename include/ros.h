#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoTcpHardware.h"

#include "NATIVE_hardware.h"

    namespace ros
{
    // default is 25, 25, 512, 512
    typedef NodeHandle_<NATIVE_hardware, 25, 25, 2048, 2048> NodeHandle;

    // This is legal too and will use the default 25, 25, 512, 512
    //typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif
