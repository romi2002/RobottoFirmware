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
#include <EEPROM.h>

#include "Tasks/UseTimeTask/UseTimeTask.h"
#include "Tasks/IMUTask/IMUTask.h"

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

CONSOLE_COMMAND_DEF(clear_eeprom, "Clears data in EEPROM");

static void clear_eeprom_command_handler(const clear_eeprom_args_t* args){
    ConsoleTask::serial->println("> Clear contents in EEPROM?");
    ConsoleTask::serial->println("> ");
    while(!ConsoleTask::serial->available());
    char response = ConsoleTask::serial->read();
    if(response == 'y' || response == 'Y'){
        ConsoleTask::serial->println("> Clearing EEPROM");
        for(int i = 0; i < EEPROM.length(); ++i){
            EEPROM.write(i, 0);
        }
        ConsoleTask::serial->println("> Finished clearing EEPROM");
    } else {
        ConsoleTask::serial->println("> Stopping...");
    }
}

CONSOLE_COMMAND_DEF(get_usetime, "Prints use time (in minutes)");

static void get_usetime_command_handler(const get_usetime_args_t * args){
    ConsoleTask::serial->print(UseTimeTask::getElapsedTime() * 5.0);
    ConsoleTask::serial->println(" mins");
}

CONSOLE_COMMAND_DEF(enable_rpy_print, "Enable printing RPY to SerialUSB2", CONSOLE_STR_ARG_DEF(status, "Boolean (true or false)"));
static void enable_rpy_print_command_handler(const enable_rpy_print_args_t *args){
    if(strcmp(args->status, "true") == 0){
        IMUTask::setPrintRPY(true);
    } else if(strcmp(args->status, "false") == 0){
        IMUTask::setPrintRPY(false);
    } else {
        ConsoleTask::serial->println("Invalid status arg!");
    }
}

CONSOLE_COMMAND_DEF(print_dhi_status, "Prints DHI status");
static void print_dhi_status_command_handler(const print_dhi_status_args_t *args){
    IMUTask::setPrintDHIStatus();
}

CONSOLE_COMMAND_DEF(reset_dhi, "Resets DHI Corrector");
static void reset_dhi_command_handler(const reset_dhi_args_t *args){
    ConsoleTask::serial->println("Resetting DHI corrector");
    IMUTask::resetDHICorrector();
}

#endif //TEENSYROSCONTROLLER_CONSOLETASK_H
