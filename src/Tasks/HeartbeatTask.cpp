//
// Created by abiel on 5/10/20.
//

#include "HeartbeatTask.hpp"
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

[[noreturn]] void heartbeatTask(void *arg){
    while(1){
        vTaskDelay((100L * configTICK_RATE_HZ) / 1000L * 2);
        digitalWrite(HEARTBEAT_PIN, HIGH);
        vTaskDelay((100L * configTICK_RATE_HZ) / 1000L * 2);
        digitalWrite(HEARTBEAT_PIN, LOW);
    }
}