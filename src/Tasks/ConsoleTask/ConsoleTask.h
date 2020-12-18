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

#include "Utils/TaskProfiler/TaskProfiler.h"

using namespace cpp_freertos;

class ConsoleTask : public Thread {
public:
    explicit ConsoleTask(TickType_t waitTime = DEFAULT_WAIT_TIME);

    static constexpr Stream *serial = &SerialUSB1;

protected:
    void Run() override;

    static void serial_write_function(const char *str);

private:
    TickType_t waitTime;

    console_init_t console;

    TaskProfiler& profiler = TaskProfiler::getInstance();
    TaskProfilerIt profilerIt;
};

CONSOLE_COMMAND_DEF(get_profiler, "Prints out profiler data");

static void get_profiler_command_handler(const get_profiler_args_t* args){
    TaskProfiler::getInstance().outputData(ConsoleTask::serial);
}

#endif //TEENSYROSCONTROLLER_CONSOLETASK_H
