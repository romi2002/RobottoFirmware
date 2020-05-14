//
// Created by abiel on 5/12/20.
//

#include "MotorTestTask.h"
#include "HAL/VNH5019.h"
#include "config.h"

void motorTestTask(void *arg){
    MotorTestTaskData *data = arg;

    while(1){
        xSemaphoreTake(data->testMutex, portMAX_DELAY);
        data->motor.set(*data->setpoint);
        xSemaphoreGive(data->testMutex);

        vTaskDelay(DEFAULT_WAIT_TIME);
    }
}