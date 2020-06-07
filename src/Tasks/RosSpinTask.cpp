#include "RosSpinTask.h"

RosSpinTask::RosSpinTask(ros::NodeHandle *nh, TickType_t waitTime) : Thread("RosSpinTask", configMINIMAL_STACK_SIZE, ROS_SPIN_TASK_PRIORITY) {
    this->nh = nh;
    this->waitTime = waitTime;

    Start();
}

[[noreturn]] void RosSpinTask::Run() {
    while (true){
        nh->spinOnce();
        vTaskDelay(waitTime);
    }
}