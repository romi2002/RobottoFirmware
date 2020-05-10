#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#define USE_USBCON

#include "PinAssignments.h"

/**
 * Tasks
 */
//#include "Tasks/RuntimeStatsPublisherTask.hpp"
#include "Tasks/RosSpinTask.hpp"
#include "Tasks/HeartbeatTask.hpp"
#include "Tasks/BatteryPublisherTask.hpp"

ros::NodeHandle nh;

void verifyTask(portBASE_TYPE ret){
    if(ret != pdPASS){
        while(1){
            delay(25);
            digitalWrite(HEARTBEAT_PIN, HIGH);
            delay(25);
            digitalWrite(HEARTBEAT_PIN, LOW);
        }
    }
}

void setup(){
  pinMode(HEARTBEAT_PIN, OUTPUT);

  analogReadResolution(12);

  nh.initNode();

  /**
   * RTOS SETUP
   */
  portBASE_TYPE rosSpin = xTaskCreate(rosSpinTask, "RosSpin", 5120, &nh, 3, NULL);
  //portBASE_TYPE runtimeStat = xTaskCreate(runtimeStatsPublisherTask, "RunStatsPublisher", 2500, &nh, 2, NULL);
  portBASE_TYPE batteryTask = xTaskCreate(batteryPublisherTask, "BatPublisher", configMINIMAL_STACK_SIZE, &nh, 2, NULL);
  portBASE_TYPE heartTask = xTaskCreate(heartbeatTask, "HeartbeatTask", configMINIMAL_STACK_SIZE, NULL, 6, NULL);

  verifyTask(heartTask);

  vTaskStartScheduler();
}

void loop(){
  ;
}