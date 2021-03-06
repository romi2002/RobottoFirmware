#ifndef TEENSYROSCONTROLLER_ROSSPINTASK_H
#define TEENSYROSCONTROLLER_ROSSPINTASK_H

#include <ros.h>
#include "config.h"
#include "thread.hpp"
#include "Utils/TaskProfiler/TaskProfiler.h"

using namespace cpp_freertos;

class RosSpinTask : public Thread {
public:
    RosSpinTask(ros::NodeHandle *nh, TickType_t waitTime = DEFAULT_WAIT_TIME);

protected:
    void Run() override;

private:
    ros::NodeHandle *nh;
    TickType_t waitTime;

    long startTime = 0;

    TaskProfiler& profiler = TaskProfiler::getInstance();
    TaskProfilerIt profilerIt;
};

#endif