#include "RosSpinTask.hpp"

[[noreturn]] void rosSpinTask(void *arg){
    ros::NodeHandle *nh = arg;

    while (1){
        nh->spinOnce();
        vTaskDelay(DEFAULT_WAIT_TIME);
    }
}