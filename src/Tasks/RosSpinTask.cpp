#include "RosSpinTask.h"

RosSpinTask::RosSpinTask(ros::NodeHandle *nh, TickType_t waitTime) : Thread("RosSpinTask", configMINIMAL_STACK_SIZE,
                                                                            ROS_SPIN_TASK_PRIORITY) {
    this->nh = nh;
    this->waitTime = waitTime;

    Start();
}

[[noreturn]] void RosSpinTask::Run() {
    while (true) {
        //SerialUSB.print("Took: "); SerialUSB.println(millis() - startTime);
        nh->spinOnce();
        startTime = millis();
        vTaskDelay(pdMS_TO_TICKS(5));

        SerialUSB.print("Spin took: "); SerialUSB.println(millis()-startTime);
        startTime = millis();
    }
}