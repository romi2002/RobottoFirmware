#ifndef TEENSYROSCONTROLLER_RUNTIMESTATSPUBLISHERTASK_H
#define TEENSYROSCONTROLLER_RUNTIMESTATSPUBLISHERTASK_H

#include <ros.h>
#include <std_msgs/String.h>
#include "config.h"

static std_msgs::String runtimeStats_msg;
static ros::Publisher runtimeStats("runtimeStats", &runtimeStats_msg);

void runtimeStatsPublisherTask(void *arg);

#endif