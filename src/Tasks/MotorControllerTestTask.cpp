//
// Created by abiel on 5/12/20.
//

#include <functional>
#include "MotorControllerTestTask.h"
#include <sstream>

MotorControllerTestTask::MotorControllerTestTask(const std::string &name, ros::NodeHandle *nh,
                                                 MotorController *controller,
                                                 TickType_t waitTime) : Thread(name, 256,
                                                                               MOTOR_OUTPUT_TEST_TASK_PRIORITY) {
    this->nh = nh;
    this->controller = controller;

    setpointTopic = name + "_setpoint";
    encoderPositionTopic = name + "_position";
    encoderVelocityTopic = name + "_velocity";
    velPIDStatusTopic = name + "_velocityPIDStatus";
    velPIDConfigTopic = name + "_velocityPIDConfig";
    posPIDStatusTopic = name + "_positionPIDStatus";
    posPIDConfigTopic = name + "_positionPIDConfig";

    this->waitTime = waitTime;

    this->setpointLock = new ReadWriteLockPreferReader();

    this->setpointSubscriber = new ros::Subscriber<std_msgs::Float32, MotorControllerTestTask>(setpointTopic.data(),
                                                                                               &MotorControllerTestTask::setpointSubscriberCb,
                                                                                               this);
    this->velPIDConfigSub = new ros::Subscriber<control_msgs::PidState, MotorControllerTestTask>(
            velPIDConfigTopic.data(), &MotorControllerTestTask::velPIDConfigSubCb, this);

    this->posPIDConfigSub = new ros::Subscriber<control_msgs::PidState, MotorControllerTestTask>(
            posPIDConfigTopic.data(), &MotorControllerTestTask::posPIDConfigSubCb, this);

    nh->subscribe(*setpointSubscriber);
    nh->subscribe(*velPIDConfigSub);
    nh->subscribe(*posPIDConfigSub);

    positionPublisher = new ros::Publisher(encoderPositionTopic.data(), &positionMsg);
    velocityPublisher = new ros::Publisher(encoderVelocityTopic.data(), &velocityMsg);

    velPIDStatusPublisher = new ros::Publisher(velPIDStatusTopic.data(),
                                               &velPIDStatusMsg);

    posPIDStatusPublisher = new ros::Publisher(posPIDStatusTopic.data(),
                                               &posPIDStatusMsg);

    nh->advertise(*positionPublisher);
    nh->advertise(*velocityPublisher);
    nh->advertise(*velPIDStatusPublisher);
    nh->advertise(*posPIDStatusPublisher);

    Start();
}

[[noreturn]] void MotorControllerTestTask::Run() {
    int32_t timesUpdated = 0;

    while (true) {
        setpointLock->ReaderLock();
        controller->set(setpoint, MotorControlMode::PERCENTAGE);
        setpointLock->ReaderUnlock();

        positionMsg.data = controller->getPosition();
        positionPublisher->publish(&positionMsg);

        velocityMsg.data = controller->getVelocity();
        velocityPublisher->publish(&velocityMsg);

        /**
         * Reduce update frequency for non essential topics
         */
        if (timesUpdated >= 10) {
            velPIDStatusMsg = controller->getVelPIDStatus();
            velPIDStatusPublisher->publish(&velPIDStatusMsg);

            posPIDStatusMsg = controller->getPosPIDStatus();
            posPIDStatusPublisher->publish(&posPIDStatusMsg);

            timesUpdated = 0;
        }

        timesUpdated++;
        vTaskDelay(waitTime);
    }
}

void MotorControllerTestTask::setpointSubscriberCb(const std_msgs::Float32 &msg) {
    setpointLock->WriterLock();
    setpoint = msg.data;
    setpointLock->WriterUnlock();
}

void MotorControllerTestTask::velPIDConfigSubCb(const control_msgs::PidState &msg) {
    PIDConfig newConfig = PIDControllerROS::getPIDConfig(msg);
    PIDConfig oldConfig = controller->getVelPIDConfig();
    if (newConfig != oldConfig) {
        oldConfig.p = newConfig.p;
        oldConfig.i = newConfig.i;
        oldConfig.d = newConfig.d;
        controller->setVelPIDConfig(oldConfig);
    }
}

void MotorControllerTestTask::posPIDConfigSubCb(const control_msgs::PidState &msg) {
    PIDConfig newConfig = PIDControllerROS::getPIDConfig(msg);
    PIDConfig oldConfig = controller->getPosPIDConfig();
    if (newConfig != oldConfig) {
        oldConfig.p = newConfig.p;
        oldConfig.i = newConfig.i;
        oldConfig.d = newConfig.d;
        controller->setPosPIDConfig(oldConfig);
    }
}

MotorControllerTestTask::~MotorControllerTestTask() {
    delete (setpointLock);
    //delete(setpointSubscriber);
    delete (positionPublisher);
    delete (velocityPublisher);
}