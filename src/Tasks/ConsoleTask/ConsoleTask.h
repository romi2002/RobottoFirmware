//
// Created by abiel on 12/17/20.
//

#ifndef TEENSYROSCONTROLLER_CONSOLETASK_H
#define TEENSYROSCONTROLLER_CONSOLETASK_H

extern "C"{
#include "anchor/console/console.h"
};

#include "config.h"
#include "PinAssignments.h"

#include "thread.hpp"

using namespace cpp_freertos;

class ConsoleTask : public Thread {
public:
    explicit ConsoleTask(TickType_t waitTime = DEFAULT_WAIT_TIME);

protected:
    void Run() override;

    static void serial_write_function(const char *str);

private:
    TickType_t waitTime;
    static constexpr Stream *serial = &SerialUSB1;

    console_init_t console;
};

#endif //TEENSYROSCONTROLLER_CONSOLETASK_H
