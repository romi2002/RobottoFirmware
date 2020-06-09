//
// Created by abiel on 5/12/20.
//

#ifndef TEENSYROSCONTROLLER_MOTORCONTROLLERTESTTASK_H
#define TEENSYROSCONTROLLER_MOTORCONTROLLERTESTTASK_H

#include <FreeRTOS_TEENSY4.h>
#include "HAL/MotorController.h"

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "config.h"

#include "thread.hpp"
#include "read_write_lock.hpp"
#include "PID/PIDControllerRos.h"

using namespace cpp_freertos;

class MotorControllerTestTask : public Thread {
public:
    MotorControllerTestTask(const std::string &name, ros::NodeHandle *nh, MotorController *controller,
                            TickType_t waitTime = DEFAULT_WAIT_TIME);

    ~MotorControllerTestTask() override;

protected:
    void Run() override;

    void setpointSubscriberCb(const std_msgs::Float32 &msg);

    void velPIDConfigSubCb(const control_msgs::PidState &msg);
    void posPIDConfigSubCb(const control_msgs::PidState &msg);

private:
    ros::NodeHandle *nh;
    MotorController *controller;

    TickType_t waitTime;

    float setpoint{0};
    ReadWriteLock *setpointLock;

    ros::Subscriber<std_msgs::Float32, MotorControllerTestTask> *setpointSubscriber;

    ros::Subscriber<control_msgs::PidState, MotorControllerTestTask> *velPIDConfigSub;
    ros::Subscriber<control_msgs::PidState, MotorControllerTestTask> *posPIDConfigSub;

    std_msgs::Float32 positionMsg;
    std_msgs::Float32 velocityMsg;

    ros::Publisher *positionPublisher;
    ros::Publisher *velocityPublisher;

    control_msgs::PidState velPIDStatusMsg;
    ros::Publisher *velPIDStatusPublisher;

    control_msgs::PidState posPIDStatusMsg;
    ros::Publisher *posPIDStatusPublisher;

    /**
     * Publisher / Subscriber topic names
     */
    std::string setpointTopic, encoderPositionTopic, encoderVelocityTopic;
    std::string velPIDStatusTopic, velPIDConfigTopic;
    std::string posPIDStatusTopic, posPIDConfigTopic;
};

#endif //TEENSYROSCONTROLLER_MOTORCONTROLLERTESTTASK_H
