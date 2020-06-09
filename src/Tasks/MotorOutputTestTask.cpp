//
// Created by abiel on 6/7/20.
//

#include "MotorOutputTestTask.h"

MotorOutputTestTask::MotorOutputTestTask(VNH5019 *motor, TickType_t waitTime) : Thread("MotorOutputTestTask", 256,
                                                                                       MOTOR_OUTPUT_TEST_TASK_PRIORITY) {
    this->motor = motor;
    this->waitTime = waitTime;

    Start();
}

[[noreturn]] void MotorOutputTestTask::Run() {
    double setpoint = 0;

    while (true) {
        motor->set(setpoint);
        setpoint += 0.001;
        if (setpoint > .1) setpoint = 0;

        vTaskDelay(waitTime);
    }
}