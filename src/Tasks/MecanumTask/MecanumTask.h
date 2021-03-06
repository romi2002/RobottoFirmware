//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_MECANUMTASK_H
#define TEENSYROSCONTROLLER_MECANUMTASK_H

#include "Kinematics/MecanumKinematics.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "Odometry/ExponentialOdometry.h"

#include "sensor_msgs/JointState.h"

#include <thread.hpp>
#include "config.h"

#include <HAL/MotorController.h>

#include <ros.h>

#include "Utils/TaskProfiler/TaskProfiler.h"
#include "Tasks/TCPSocketTask/TCPSocketTask.h"

class MecanumTask : public cpp_freertos::Thread {
public:
    MecanumTask(const MotorControllerConfig &config, ros::NodeHandle *nh, double wheelDiameter,
                TickType_t waitTime = DEFAULT_WAIT_TIME);

    /**
     * Returns in rads traveled
     * @return
     */
    MecanumWheelVelocities getWheelPositions() const;

    /**
     * Returns in rads/sec
     * @return
     */
    MecanumWheelVelocities getWheelVelocities() const;

protected:
    void Run() override;

    void twistSubsriberCb(const geometry_msgs::Twist &msg);

private:
    void writeToMotors(const MecanumWheelVelocities &vel, MotorControlMode controlMode);

private:
    const double encoderCodes = 1900.0;

    double wheelDiameter, wheelCircumference;

    MotorControllerConfig m_config;

    MecanumWheelValues<MotorController *> controllers;

    MecanumKinematics *kinematics;

    TickType_t waitTime;

    cpp_freertos::ReadWriteLock *twistLock;
    Twist2D currentTarget;

    ExponentialOdometry *odometry;
    elapsedMicros lastUpdate;

    ros::NodeHandle *nh;

    TaskProfiler& profiler = TaskProfiler::getInstance();
    TaskProfilerIt profilerIt;

    OutData &outData{TCPSocketTask::dataOut};

    InData &inData{TCPSocketTask::dataIn};
};


#endif //TEENSYROSCONTROLLER_MECANUMTASK_H
