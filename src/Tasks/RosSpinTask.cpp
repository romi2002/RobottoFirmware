#include "RosSpinTask.h"
#include <TeensyDebug.h>
#pragma GCC optimize ("O0")

RosSpinTask::RosSpinTask(ros::NodeHandle *nh, TickType_t waitTime) : Thread("RosSpinTask", configMINIMAL_STACK_SIZE,
                                                                            ROS_SPIN_TASK_PRIORITY) {
    this->nh = nh;
    this->waitTime = waitTime;

    Start();
}

[[noreturn]] void RosSpinTask::Run() {
    while (true) {
        nh->spinOnce();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}