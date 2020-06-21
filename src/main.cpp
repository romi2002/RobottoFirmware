#include <LinkerFix.h>
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#define USE_USBCON

#include "PinAssignments.h"

/**
 * Tasks
 */
//#include "Tasks/RuntimeStatsPublisherTask.hpp"
#include "Tasks/RosSpinTask.h"
#include "Tasks/HeartbeatTask.h"
#include "Tasks/BatteryPublisherTask.h"
#include "Tasks/MotorControllerTestTask.h"
#include "Tasks/MotorOutputTestTask.h"
#include "HAL/MotorController.h"

#include "HAL/VNH5019.h"

#include <std_msgs/Float64.h>

#include "thread.hpp"

ros::NodeHandle nh;

void setup() {
    analogReadResolution(12);

    nh.initNode();

    MotorControllerConfig controllerConfig;
    controllerConfig.vnh5019PinDefinitions = PinAssignments::getMotor1Driver();
    controllerConfig.encoderPinDefinitions = PinAssignments::getMotor1Encoder();
    PIDConfig positionConfig;
    positionConfig.p = 0.1;
    positionConfig.d = 0.002;
    //velocityConfig.i = 0.0001;
    positionConfig.deadband = 0.1;
    positionConfig.enableRampRate = true;
    positionConfig.rampRate = 1000;
    controllerConfig.positionPIDConfig = positionConfig;

    HeartbeatTask heartbeatTask;
    BatteryPublisherTask batteryPublisherTask(&nh);
    RosSpinTask rosSpinTask(&nh);

    MotorController controller("TestController", controllerConfig, pdMS_TO_TICKS(10));
    MotorControllerTestTask motorOutputTestTask("TestMotor", &nh, &controller);

    Thread::StartScheduler();
}

void loop() {
    ;
}