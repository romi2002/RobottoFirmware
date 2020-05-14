//
// Created by abiel on 5/12/20.
//

#ifndef TEENSYROSCONTROLLER_MOTORTESTTASK_H
#define TEENSYROSCONTROLLER_MOTORTESTTASK_H

#include <FreeRTOS_TEENSY4.h>
#include "HAL/VNH5019.h"

struct MotorTestTaskData {
    SemaphoreHandle_t testMutex{};
    double *setpoint{};
    VNH5019 motor;
};

void motorTestTask(void *arg);


#endif //TEENSYROSCONTROLLER_MOTORTESTTASK_H
