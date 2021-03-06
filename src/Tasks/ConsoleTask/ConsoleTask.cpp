//
// Created by abiel on 12/17/20.
//

#include "ConsoleTask.h"

ConsoleTask::ConsoleTask(TickType_t waitTime) : Thread("ConsoleTask", 512, CONSOLE_TASK_PRIORITY) {
    this->waitTime = waitTime;

    //Init console
    console = {
            .write_function = ConsoleTask::serial_write_function,
    };
    console_init(&console);

    //Register console command
    console_command_register(get_profiler);
    console_command_register(clear_eeprom);
    console_command_register(get_usetime);
    console_command_register(enable_rpy_print);
    console_command_register(print_dhi_status);
    console_command_register(reset_dhi);

    profilerIt = profiler.initProfiler("ConsoleTask");
    Start();
}

void ConsoleTask::serial_write_function(const char *str) {
    serial->print(str);
}

[[noreturn]] void ConsoleTask::Run() {
    uint8_t buffer[CONSOLE_MAX_LINE_LENGTH * 2];

    for(;;){
        size_t len = 0;

        while(serial->available()){
            char c = serial->read();
            buffer[len++] = c;
            if(c == '\n'){
                //New line
                break;
            }
        }

        console_process(buffer, len);

        TaskProfiler::updateProfiler(profilerIt);
        vTaskDelay(waitTime);
    }
}