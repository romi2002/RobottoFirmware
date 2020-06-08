//
// Created by abiel on 5/12/20.
//

#ifndef TEENSYROSCONTROLLER_MOTORTESTTASK_H
#define TEENSYROSCONTROLLER_MOTORTESTTASK_H

#include <FreeRTOS_TEENSY4.h>
#include "HAL/MotorController.h"

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "config.h"

#include "thread.hpp"
#include "read_write_lock.hpp"

using namespace cpp_freertos;

class MotorTestTask : public Thread{
public:
    MotorTestTask(const std::string &name, ros::NodeHandle *nh, MotorController *controller, TickType_t waitTime = DEFAULT_WAIT_TIME);

    void setpointSubscriberCb(const std_msgs::Float32& msg);

    ~MotorTestTask();

protected:
    void Run() override;


private:
    std::string name;
    ros::NodeHandle *nh;
    MotorController *controller;

    TickType_t waitTime;

    float targetSetpoint{0};
    ReadWriteLock *setpointLock;

    ros::Subscriber<std_msgs::Float32, MotorTestTask> *setpointSubscriber;

    std_msgs::Float32 motorEncoderPosition_msg;
    std_msgs::Float32 motorEncoderVelocity_msg;

    ros::Publisher *motorEncoderPositionPublisher;
    ros::Publisher *motorEncoderVelocityPublisher;

    /**
     * Publisher / Subscriber topic names
     */
    std::string setpointTopic, encoderPositionTopic, encoderVelocityTopic;

    const float positionAverageAlpha = 0.1;
    const float velocityAverageAlpha = 0.1;
};

#endif //TEENSYROSCONTROLLER_MOTORTESTTASK_H
