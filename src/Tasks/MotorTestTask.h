//
// Created by abiel on 5/12/20.
//

#ifndef TEENSYROSCONTROLLER_MOTORTESTTASK_H
#define TEENSYROSCONTROLLER_MOTORTESTTASK_H

#include <FreeRTOS_TEENSY4.h>
#include "HAL/VNH5019.h"

#include <ros.h>
#include <std_msgs/Int32.h>

struct MotorTestTaskData {
    SemaphoreHandle_t testMutex{};
    double *setpoint{};
    VNH5019 motor;
    uint8_t encoderChannel, phaseA_PIN, phaseB_PIN;
    ros::NodeHandle *nh;
};

static std_msgs::Int32 motorEncoderTest1_msg;
static ros::Publisher motorEncoderTest1("motorEncoder1", &motorEncoderTest1_msg);

static std_msgs::Int32 motorEncoderTest2_msg;
static ros::Publisher motorEncoderTest2("motorEncoder2", &motorEncoderTest2_msg);

[[noreturn]] void motorTestTask(void *arg);


#endif //TEENSYROSCONTROLLER_MOTORTESTTASK_H
