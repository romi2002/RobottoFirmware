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
#include "Tasks/MotorTestTask.h"

#include "HAL/VNH5019.h"

#include <std_msgs/Float64.h>

#include "thread.hpp"

ros::NodeHandle nh;

SemaphoreHandle_t  testMutex;
double currentSetpoint = 0;

void messageCb(const std_msgs::Float64& msg){
    xSemaphoreTake(testMutex, portMAX_DELAY);
    currentSetpoint = msg.data;
    xSemaphoreGive(testMutex);
}

ros::Subscriber<std_msgs::Float64> testSubcriber("motorTest", &messageCb);

void verifyTask(portBASE_TYPE ret) {
    if (ret != pdPASS) {
        while (1) {
            delay(25);
            digitalWrite(HEARTBEAT_PIN, HIGH);
            delay(25);
            digitalWrite(HEARTBEAT_PIN, LOW);
        }
    }
}

void setup() {
    pinMode(HEARTBEAT_PIN, OUTPUT);

    analogReadResolution(12);

    testMutex = xSemaphoreCreateMutex();

    nh.initNode();
    nh.subscribe(testSubcriber);

    /**
     * RTOS SETUP
     */
/*    portBASE_TYPE rosSpin = xTaskCreate(rosSpinTask, "RosSpin", 5120, &nh, 3, NULL);
    //portBASE_TYPE runtimeStat = xTaskCreate(runtimeStatsPublisherTask, "RunStatsPublisher", 2500, &nh, 2, NULL);
    portBASE_TYPE batteryTask = xTaskCreate(batteryPublisherTask, "BatPublisher", configMINIMAL_STACK_SIZE, &nh, 2,
                                            NULL);
    portBASE_TYPE heartTask = xTaskCreate(heartbeatTask, "HeartbeatTask", configMINIMAL_STACK_SIZE, NULL, 6, NULL);

    VNH5019_PinDefinitions definitions;
    definitions.PWM = PIN_A1;
    definitions.IN_A = 11;
    definitions.IN_B = 10;
    definitions.DIAG_A = 20;
    definitions.DIAG_B = 21;
    definitions.CS = PIN_A2;

    VNH5019 testMotor(definitions);
    MotorTestTaskData data1;
    data1.testMutex = testMutex;
    data1.motor = testMotor;
    data1.setpoint = &currentSetpoint;
    data1.nh = &nh;
    data1.encoderChannel = 1;
    data1.phaseA_PIN = 2;
    data1.phaseB_PIN = 3;

    definitions.PWM = PIN_A0;
    definitions.IN_A = 13;
    definitions.IN_B = 12;
    definitions.DIAG_A = 6;
    definitions.DIAG_B = 9;
    definitions.CS = PIN_A3;

    VNH5019 testMotor2(definitions);

    MotorTestTaskData data2;
    data2.testMutex = testMutex;
    data2.motor = testMotor2;
    data2.setpoint = &currentSetpoint;
    data2.nh = &nh;
    data2.encoderChannel = 2;
    data2.phaseA_PIN = 4;
    data2.phaseB_PIN = 5;

    portBASE_TYPE motorTask = xTaskCreate(motorTestTask, "MotorTestTask", 128, &data1, 7, NULL);
    portBASE_TYPE motorTask2 = xTaskCreate(motorTestTask, "MotorTestTask2", 128, &data2, 7, NULL);

    verifyTask(heartTask);

    vTaskStartScheduler();*/

    HeartbeatTask heartbeatThread;

    Thread::StartScheduler();
}

void loop() {
    ;
}