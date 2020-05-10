#ifndef TEENSYROSCONTROLLER_BATTERYPUBLISHERTASK_HPP
#define TEENSYROSCONTROLLER_BATTERYPUBLISHERTASK_HPP

#include "config.h"
#include "PinAssignments.h"

#include <ros.h>
#include <std_msgs/Float32.h>

static std_msgs::Float32 currentSensorVoltageMsg;
static std_msgs::Float32 currentSensorMsg;
static std_msgs::Float32 batteryVoltageMsg;

static ros::Publisher currentSensorVoltagePub("currentSensorVoltage", &currentSensorVoltageMsg);
static ros::Publisher currentSensorPub("currentSensor", &currentSensorMsg);
static ros::Publisher batteryVoltagePub("batteryVoltage", &batteryVoltageMsg);

void batteryPublisherTask(void *arg);

#endif