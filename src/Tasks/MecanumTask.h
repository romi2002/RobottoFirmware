//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_MECANUMTASK_H
#define TEENSYROSCONTROLLER_MECANUMTASK_H

#include "Kinematics/MecanumKinematics.h"
#include "geometry_msgs/Twist.h"

#include <thread.hpp>
#include "config.h"

#include <HAL/MotorController.h>

#include <ros.h>

class MecanumTask : public cpp_freertos::Thread {
public:
    MecanumTask(const MotorControllerConfig &config, ros::NodeHandle *nh, TickType_t waitTime = DEFAULT_WAIT_TIME);

protected:
    void Run() override;

    void twistSubsriberCb(const geometry_msgs::Twist &msg);

private:
    void writeToMotors(const MecanumWheelVelocities &vel, MotorControlMode controlMode);

private:
    MotorControllerConfig m_config;

    MecanumWheelValues<MotorController*> controllers;

    MecanumKinematics *kinematics;

    TickType_t waitTime;

    ros::Subscriber<geometry_msgs::Twist, MecanumTask> *twistSetpointSubscriber;
    cpp_freertos::ReadWriteLock *twistLock;
    Twist2D currentTarget;

    ros::NodeHandle *nh;
};


#endif //TEENSYROSCONTROLLER_MECANUMTASK_H
