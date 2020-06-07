//
// Created by abiel on 5/12/20.
//

#include "MotorTestTask.h"
#include "HAL/VNH5019.h"
#include "config.h"
#include "QuadEncoder_Teensy4.h"

[[noreturn]] void motorTestTask(void *arg){
    MotorTestTaskData *data = static_cast<MotorTestTaskData *>(arg);

    QuadEncoder encoder(data->encoderChannel, data->phaseA_PIN, data->phaseB_PIN);
    encoder.setInitConfig();
    encoder.EncConfig.INDEXTriggerMode = RISING_EDGE;

    encoder.init();

    if(data->encoderChannel == 1){
        data->nh->advertise(motorEncoderTest1);
    } else if(data->encoderChannel == 2){
        data->nh->advertise(motorEncoderTest2);
    }

    while(1){
        xSemaphoreTake(data->testMutex, portMAX_DELAY);
        data->motor.set(*data->setpoint);
        xSemaphoreGive(data->testMutex);

        if(data->encoderChannel == 1){
            motorEncoderTest1_msg.data = encoder.read();
            motorEncoderTest1.publish(&motorEncoderTest1_msg);
        } else if(data->encoderChannel == 2){
            motorEncoderTest2_msg.data = encoder.read();
            motorEncoderTest2.publish(&motorEncoderTest2_msg);
        }

        vTaskDelay(DEFAULT_WAIT_TIME);
    }
}