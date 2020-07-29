#include <LinkerFix.h>
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#define USE_USBCON
#include "log.h"
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
#include "Tasks/MecanumTask.h"
#include "HAL/MotorController.h"

#include "HAL/VNH5019.h"

#include <std_msgs/Float64.h>

#include "thread.hpp"
#include "read_write_lock.hpp"

#include "Adafruit_MCP23017.h"
#include "ADC128D818.h"

ros::NodeHandle nh;

Adafruit_MCP23017 mcp;
cpp_freertos::ReadWriteLockPreferWriter *mcpLock;

void setup() {
    SerialUSB1.begin(115200);

    analogReadResolution(12);
    analogReadAveraging(4);
    pinMode(PinAssignments::DEBUG_PIN, OUTPUT);

    //Wire.begin();

    /**
     * ADC128D818 Init
     */
    /*adc128D818 = new ADC128D818(0x37);
    adc128D818->setReferenceMode(EXTERNAL_REF);
    adc128D818->setReference(3.3);
    adc128D818->setDisabledMask(0b00000010);
    adc128D818->begin();*/

    nh.initNode();

    /**
     * MCP Init
     */
    mcpLock = new cpp_freertos::ReadWriteLockPreferWriter();
    mcp.begin(0x27);

    MotorControllerConfig controllerConfig{};
    controllerConfig.vnh5019PinDefinitions = PinAssignments::getMotor4Driver();
    controllerConfig.encoderPinDefinitions = PinAssignments::getMotor4Encoder();
    PIDConfig positionConfig;
    positionConfig.p = 0.1;
    positionConfig.d = 0.002;
    //velocityConfig.i = 0.0001;
    positionConfig.deadband = 0.1;
    positionConfig.enableRampRate = true;
    positionConfig.rampRate = 1000;
    controllerConfig.positionPIDConfig = positionConfig;
    controllerConfig.mcp = &mcp;
    controllerConfig.mcpLock = mcpLock;

    HeartbeatTask heartbeatTask;
    //ADCTestTask adcTestTask(adc128D818);
    BatteryPublisherTask batteryPublisherTask(&nh);
    RosSpinTask rosSpinTask(&nh);

    MecanumTask mecanumTask(controllerConfig, &nh);

    //MotorController controller("TestController", controllerConfig, pdMS_TO_TICKS(10));
    //controller.set(0.0, MotorControlMode::PERCENTAGE);
   // MotorControllerTestTask motorOutputTestTask("TestMotor", &nh, &controller);

    Thread::StartScheduler();
}

void loop() {
    //
}