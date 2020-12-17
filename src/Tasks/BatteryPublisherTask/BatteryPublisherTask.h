#ifndef TEENSYROSCONTROLLER_BATTERYPUBLISHERTASK_H
#define TEENSYROSCONTROLLER_BATTERYPUBLISHERTASK_H

#include "config.h"
#include "PinAssignments.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#include "thread.hpp"

using namespace cpp_freertos;

class BatteryPublisherTask : public Thread {
public:
    explicit BatteryPublisherTask(ros::NodeHandle *nh, TickType_t waitTime = DEFAULT_WAIT_TIME);

protected:
    void Run() override;

private:
    ros::NodeHandle *nh;
    TickType_t waitTime;

    ros::Publisher currentSensorVoltagePub;
    ros::Publisher energyUsedPub;
    ros::Publisher batteryStatePub;

    sensor_msgs::BatteryState batteryStateMsg;
    std_msgs::Float32 currentSensorVoltageMsg;
    std_msgs::Float32 energyUsedMsg;

    double energyUsed;
    elapsedMillis dt;
};

#endif