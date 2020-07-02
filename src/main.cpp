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
#include "read_write_lock.hpp"

#include "Adafruit_MCP23017.h"
#include <ArduinoLog.h>

ros::NodeHandle nh;

Adafruit_MCP23017 mcp;
cpp_freertos::ReadWriteLockPreferWriter *mcpLock;

void printTimestamp(Print* _logOutput) {
    char c[12];
    int m = sprintf(c, "%10lu ", millis());
    _logOutput->print(c);
}

void printNewline(Print* _logOutput) {
    _logOutput->print('\n');
}


void setup() {
    SerialUSB1.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &SerialUSB1);
    Log.setSuffix(printNewline);

    analogReadResolution(12);
    pinMode(PinAssignments::DEBUG_PIN, OUTPUT);

    nh.initNode();

    /**
     * MCP Init
     */
    mcpLock = new cpp_freertos::ReadWriteLockPreferWriter();
    mcp.begin(0x27);

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
    controllerConfig.mcp = &mcp;
    controllerConfig.mcpLock = mcpLock;

    HeartbeatTask heartbeatTask;
    //BatteryPublisherTask batteryPublisherTask(&nh);
    //RosSpinTask rosSpinTask(&nh);

    MotorController controller("TestController", controllerConfig, pdMS_TO_TICKS(10));
    controller.set(0.0, MotorControlMode::PERCENTAGE);
    //MotorControllerTestTask motorOutputTestTask("TestMotor", &nh, &controller);

    Log.notice(F(CR "******************************************" CR));                     // Info string with Newline
    Log.notice(  "***          Logging example                " CR);                       // Info string in flash memory
    Log.notice(F("******************* ")); Log.notice("*********************** " CR);

    Thread::StartScheduler();
}

void loop() {
    //
}