#include "RuntimeStatsPublisherTask.h"

[[noreturn]] void runtimeStatsPublisherTask(void *arg) {
    ros::NodeHandle *nh = arg;

    nh->advertise(runtimeStats);

    char buffer[1024];

    while (1){
        vTaskGetRunTimeStats(buffer);
        runtimeStats_msg.data = buffer;
        runtimeStats.publish(&runtimeStats_msg);

        vTaskDelay(DEFAULT_WAIT_TIME);
    }
}