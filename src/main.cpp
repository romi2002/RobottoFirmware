#define ROSSERIAL_ARDUINO_TCP

#include <NativeEthernet.h>
#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOS_TEENSY4.h>

#include <ros.h>

#include "log.h"
#include "PinAssignments.h"

#include "Tasks/TCPSocketTask/TCPSocketTask.h"

#ifndef UNIT_TEST
/**
 * Tasks
 */
//#include "Tasks/RuntimeStatsPublisherTask.hpp"
#include "Tasks/RosSpinTask/RosSpinTask.h"
#include "Tasks/HeartbeatTask/HeartbeatTask.h"
#include "Tasks/BatteryPublisherTask/BatteryPublisherTask.h"
#include "Tasks/MotorControllerTestTask/MotorControllerTestTask.h"
#include "Tasks/MotorOutputTestTask/MotorOutputTestTask.h"
#include "Tasks/MecanumTask/MecanumTask.h"
#include "Tasks/IMUTask/IMUTask.h"
#include "Tasks/UseTimeTask/UseTimeTask.h"
#include "Tasks/ConsoleTask/ConsoleTask.h"
#include "HAL/MotorController.h"

#include "HAL/VNH5019.h"

#include <std_msgs/Float64.h>

#include "thread.hpp"
#include "read_write_lock.hpp"

#include "Adafruit_MCP23017.h"
#include "ADC128D818.h"

byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0xAD};

ros::NodeHandle nh;

Adafruit_MCP23017 mcp;
cpp_freertos::ReadWriteLockPreferWriter *i2cLock;
#include "TeensyDebug.h"

void setup() {
    //SerialUSB1.begin(115200);
    SerialUSB.begin(115200);
    SerialUSB1.begin(115200);
    SerialUSB2.begin(115200);
    Serial5.begin(576000);
    Serial8.begin(1000000, SERIAL_8E1);

    pinMode(0, INPUT);
    pinMode(1, INPUT);

    //Serial8.attachRts(32);
    //Serial8.attachCts(43);

    analogReadResolution(12);
    analogReadAveraging(4);
    pinMode(PinAssignments::DEBUG_PIN, OUTPUT);
    digitalWrite(PinAssignments::DEBUG_PIN, LOW);

    pinMode(PinAssignments::IMU_INT_PIN, INPUT_PULLDOWN);
    pinMode(PinAssignments::IMU_RST_PIN, OUTPUT);

    i2cLock = new cpp_freertos::ReadWriteLockPreferWriter();

    Wire.begin();
    mcp.begin(0x27);

    Wire.setClock(100);

    /**
    * MCP Init
    */

    delay(2000);

    IMUTask imuTask(i2cLock);

    /**
     * ADC128D818 Init
     */
    /*adc128D818 = new ADC128D818(0x37);
    adc128D818->setReferenceMode(EXTERNAL_REF);
    adc128D818->setReference(3.3);
    adc128D818->setDisabledMask(0b00000010);
    adc128D818->begin();*/

    MotorControllerConfig controllerConfig{};
    controllerConfig.vnh5019PinDefinitions = PinAssignments::getMotor4Driver();
    controllerConfig.encoderPinDefinitions = PinAssignments::getMotor4Encoder();
    PIDConfig positionConfig;
    positionConfig.p = 0.1;
    positionConfig.d = 0.002;
    //velocityConfig.i = 0.0001;
    positionConfig.deadband = 0.15;
    positionConfig.enableRampRate = true;
    positionConfig.rampRate = 50;
    controllerConfig.positionPIDConfig = positionConfig;
    controllerConfig.mcp = &mcp;
    controllerConfig.i2cLock = i2cLock;

    HeartbeatTask heartbeatTask;
    TCPSocketTask webSocketTask;
    //ADCTestTask adcTestTask(adc128D818);
    //BatteryPublisherTask batteryPublisherTask(&nh);
    //RosSpinTask rosSpinTask(&nh, pdMS_TO_TICKS(10));

    MecanumTask mecanumTask(controllerConfig, &nh, 0.075);

    //ConsoleTask consoleTask;
    //UseTimeTask useTimeTask;

    //MotorController controller("TestController", controllerConfig, pdMS_TO_TICKS(10));
    //controller.set(0.0, MotorControlMode::PERCENTAGE);
    // MotorControllerTestTask motorOutputTestTask("TestMotor", &nh, &controller);

    Thread::StartScheduler();
}

void loop() {
    ;
}
#endif