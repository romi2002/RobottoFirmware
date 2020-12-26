#include "RosSpinTask.h"



RosSpinTask::RosSpinTask(ros::NodeHandle *nh, TickType_t waitTime) : Thread("RosSpinTask", configMINIMAL_STACK_SIZE,
                                                                            ROS_SPIN_TASK_PRIORITY) {
    this->nh = nh;
    this->waitTime = waitTime;

    profilerIt = profiler.initProfiler("RosSpinTask");
    Start();
}

[[noreturn]] void RosSpinTask::Run() {
    //vTaskDelay(pdMS_TO_TICKS(1000));
    auto &usbHost(usb_host_serial::getInstance().usbHost);
    auto &userial(usb_host_serial::getInstance().userial);

    while (true) {
        //SerialUSB.print("Took: "); SerialUSB.println(millis() - startTime);
        nh->spinOnce();
        startTime = millis();

        TaskProfiler::updateProfiler(profilerIt);
        startTime = millis();

        usbHost.Task();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}