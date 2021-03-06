//
// Created by abiel on 5/10/20.
//

#ifndef TEENSYROSCONTROLLER_HEARTBEATTASK_H
#define TEENSYROSCONTROLLER_HEARTBEATTASK_H

#include "config.h"
#include "PinAssignments.h"
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include "thread.hpp"
#include "Watchdog_t4.h"
#include "Utils/TaskProfiler/TaskProfiler.h"

using namespace cpp_freertos;

class HeartbeatTask : public Thread {
public:
    explicit HeartbeatTask(TickType_t tickDelay = 200);

protected:
    TickType_t tickDelay;
    WDT_T4 <WDT1> wdt;

    [[noreturn]] void Run() override;

    TaskProfiler& profiler = TaskProfiler::getInstance();
    TaskProfilerIt profilerIt;
};

#endif //TEENSYROSCONTROLLER_HEARTBEATTASK_H
