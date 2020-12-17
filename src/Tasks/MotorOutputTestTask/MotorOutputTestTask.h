//
// Created by abiel on 6/7/20.
//

#ifndef TEENSYROSCONTROLLER_MOTOROUTPUTTESTTASK_H
#define TEENSYROSCONTROLLER_MOTOROUTPUTTESTTASK_H

#include "config.h"
#include "HAL/VNH5019.h"
#include "thread.hpp"

using namespace cpp_freertos;

class MotorOutputTestTask : public Thread {
public:
    MotorOutputTestTask(VNH5019 *motor, TickType_t waitTime = DEFAULT_WAIT_TIME);

protected:
    void Run() override;

private:
    VNH5019 *motor;
    TickType_t waitTime;
};


#endif //TEENSYROSCONTROLLER_MOTOROUTPUTTESTTASK_H
