//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_MECANUMTASK_H
#define TEENSYROSCONTROLLER_MECANUMTASK_H

#include "Kinematics/MecanumKinematics.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "Odometry/ExponentialOdometry.h"

#include <thread.hpp>
#include "config.h"

#include <HAL/MotorController.h>

#include <ros.h>

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
    const double encoderCodes = 1200.0;

    double wheelDiameter, wheelCircumference;

    MotorControllerConfig m_config;

    MecanumWheelValues<MotorController *> controllers;

    MecanumKinematics *kinematics;

    TickType_t waitTime;

    ros::Subscriber <geometry_msgs::Twist, MecanumTask> *twistSetpointSubscriber;
    cpp_freertos::ReadWriteLock *twistLock;
    Twist2D currentTarget;

    ExponentialOdometry *odometry;

    geometry_msgs::Pose posePublisherMsg;
    ros::Publisher *posePublisher;

    geometry_msgs::Twist twistPublisherMsg;
    ros::Publisher *twistPublisher;

    ros::NodeHandle *nh;
};


#endif //TEENSYROSCONTROLLER_MECANUMTASK_H
