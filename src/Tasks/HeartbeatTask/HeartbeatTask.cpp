//
// Created by abiel on 5/10/20.
//

#include "HeartbeatTask.h"

HeartbeatTask::HeartbeatTask(TickType_t tickDelay) : Thread("HeartbeatTask", configMINIMAL_STACK_SIZE,
                                                            HEARTBEAT_TASK_PRIORITY) {
    this->tickDelay = tickDelay;
    pinMode(PinAssignments::HEARTBEAT_PIN, OUTPUT);

    WDT_timings_t config;
    config.timeout = 5;

    wdt.begin(config);

    profilerIt = profiler.initProfiler("HeartbeatTask");

    Start();
}

[[noreturn]] void HeartbeatTask::Run() {
    while (true) {
        vTaskDelay((tickDelay * configTICK_RATE_HZ) / 1000L);
        digitalWrite(PinAssignments::HEARTBEAT_PIN, HIGH);
        vTaskDelay((tickDelay * configTICK_RATE_HZ) / 1000L);
        digitalWrite(PinAssignments::HEARTBEAT_PIN, LOW);
        wdt.feed();
        TaskProfiler::updateProfiler(profilerIt);
    }
}