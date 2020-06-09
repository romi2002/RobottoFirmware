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
#include "PID/PIDControllerRos.h"

using namespace cpp_freertos;

class MotorTestTask : public Thread{
public:
    MotorTestTask(const std::string &name, ros::NodeHandle *nh, MotorController *controller, TickType_t waitTime = DEFAULT_WAIT_TIME);

    ~MotorTestTask() override;

protected:
    void Run() override;

    void setpointSubscriberCb(const std_msgs::Float32& msg);
    void pidVelocityStateSubscriberCb(const control_msgs::PidState &msg);

private:
    std::string name;
    ros::NodeHandle *nh;
    MotorController *controller;

    TickType_t waitTime;

    float targetSetpoint{0};
    ReadWriteLock *setpointLock;

    ros::Subscriber<std_msgs::Float32, MotorTestTask> *setpointSubscriber;
    ros::Subscriber<control_msgs::PidState, MotorTestTask> *pidVelocityStateSubscriber;
    std::string stringBuffer;

    std_msgs::Float32 motorEncoderPosition_msg;
    std_msgs::Float32 motorEncoderVelocity_msg;

    ros::Publisher *motorEncoderPositionPublisher;
    ros::Publisher *motorEncoderVelocityPublisher;

    control_msgs::PidState motorVelocityControllerStatus_msg;
    ros::Publisher *motorVelocityControllerStatusPublisher;

    /**
     * Publisher / Subscriber topic names
     */
    std::string setpointTopic, encoderPositionTopic, encoderVelocityTopic;
    std::string velocityControllerStatusTopic, velocityControllerConfigTopic;

    const float positionAverageAlpha = 0.1;
    const float velocityAverageAlpha = 0.1;
};

#endif //TEENSYROSCONTROLLER_MOTORTESTTASK_H
