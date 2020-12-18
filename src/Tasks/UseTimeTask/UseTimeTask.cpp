//
// Created by abiel on 12/18/20.
//

#include "UseTimeTask.h"
#include <EEPROM.h>

uint16_t UseTimeTask::currTime;

UseTimeTask::UseTimeTask(TickType_t waitTime) : Thread("UseTimeTask", configMINIMAL_STACK_SIZE, 0) {
    this->waitTime = waitTime;
    currTime = readFromEEPROM();

    profilerIt = TaskProfiler::getInstance().initProfiler("RosSpinTask");
    Start();
}

[[noreturn]] void UseTimeTask::Run() {
    for(;;){
        TaskProfiler::updateProfiler(profilerIt);
        vTaskDelay(waitTime);
        writeToEEPROM(++currTime);
    }
}

void UseTimeTask::writeToEEPROM(uint16_t val) {
    char byte1 = val >> 8;
    char byte2 = val & 0xFF;

    EEPROM.write(USE_TIME_LOCATION, byte1);
    EEPROM.write(USE_TIME_LOCATION + 1, byte2);
}

uint16_t UseTimeTask::readFromEEPROM() {
    char byte1 = EEPROM.read(USE_TIME_LOCATION);
    char byte2 = EEPROM.read(USE_TIME_LOCATION + 1);

    return (byte1 << 8) + byte2;
}