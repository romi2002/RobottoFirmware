//
// Created by abiel on 12/18/20.
//

#ifndef TEENSYROSCONTROLLER_USETIMETASK_H
#define TEENSYROSCONTROLLER_USETIMETASK_H

#include "config.h"
#include "thread.hpp"
#include "Utils/TaskProfiler/TaskProfiler.h"

using namespace cpp_freertos;

class UseTimeTask : public Thread {
public:
    explicit UseTimeTask(TickType_t waitTime = pdMS_TO_TICKS(5 * 60 * 1000)); //5 min
    static uint16_t getElapsedTime(){
        return currTime;
    }

protected:
    void Run() override;

private:
    static void writeToEEPROM(uint16_t val);
    static uint16_t readFromEEPROM();

    static uint16_t currTime;
    TickType_t waitTime;
    TaskProfilerIt profilerIt;
};


#endif //TEENSYROSCONTROLLER_USETIMETASK_H
