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

using namespace cpp_freertos;

class HeartbeatTask : public Thread {
public:
    explicit HeartbeatTask(TickType_t tickDelay = 200);

protected:
    TickType_t tickDelay;

    [[noreturn]] void Run() override;
};

#endif //TEENSYROSCONTROLLER_HEARTBEATTASK_H
