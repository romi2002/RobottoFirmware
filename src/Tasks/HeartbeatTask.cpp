//
// Created by abiel on 5/10/20.
//

#include "HeartbeatTask.h"
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

HeartbeatTask::HeartbeatTask(TickType_t tickDelay) : Thread("HeartbeatTask", 100, 1) {
    this->tickDelay = tickDelay;

    Start();
}

[[noreturn]] void HeartbeatTask::Run() {
    while(true){
        vTaskDelay((tickDelay * configTICK_RATE_HZ) / 1000L);
        digitalWrite(HEARTBEAT_PIN, HIGH);
        vTaskDelay((tickDelay * configTICK_RATE_HZ) / 1000L);
        digitalWrite(HEARTBEAT_PIN, LOW);
    }
}