//
// Created by abiel on 5/12/20.
//

#include <functional>
#include "MotorTestTask.h"
#include <sstream>

MotorTestTask::MotorTestTask(const std::string &name, ros::NodeHandle *nh, MotorController *controller,
                             TickType_t waitTime) : Thread(name, 256, MOTOR_OUTPUT_TEST_TASK_PRIORITY) {
    this->name = name;
    this->nh = nh;
    this->controller = controller;

    setpointTopic = name + "_setpoint";
    encoderPositionTopic = name + "_position";
    encoderVelocityTopic = name + "_velocity";
    velocityControllerStatusTopic = name + "_velocityPIDStatus";
    velocityControllerConfigTopic = name + "_velocityPIDConfig";

    this->waitTime = waitTime;

    this->setpointLock = new ReadWriteLockPreferReader();

    this->setpointSubscriber = new ros::Subscriber<std_msgs::Float32, MotorTestTask>(setpointTopic.data(),
                                                                                     &MotorTestTask::setpointSubscriberCb,
                                                                                     this);
    this->pidVelocityStateSubscriber = new ros::Subscriber<control_msgs::PidState, MotorTestTask>(
            velocityControllerConfigTopic.data(), &MotorTestTask::pidVelocityStateSubscriberCb, this);

    nh->subscribe(*setpointSubscriber);
    nh->subscribe(*pidVelocityStateSubscriber);

    motorEncoderPositionPublisher = new ros::Publisher(encoderPositionTopic.data(), &motorEncoderPosition_msg);
    motorEncoderVelocityPublisher = new ros::Publisher(encoderVelocityTopic.data(), &motorEncoderVelocity_msg);
    motorVelocityControllerStatusPublisher = new ros::Publisher(velocityControllerStatusTopic.data(),
                                                                &motorVelocityControllerStatus_msg);

    nh->advertise(*motorEncoderPositionPublisher);
    nh->advertise(*motorEncoderVelocityPublisher);
    nh->advertise(*motorVelocityControllerStatusPublisher);

    Start();
}

[[noreturn]] void MotorTestTask::Run() {
    int32_t timesUpdated = 0;

    while (true) {
        setpointLock->ReaderLock();
        controller->set(targetSetpoint, MotorControlMode::VELOCITY);
        setpointLock->ReaderUnlock();

        motorEncoderPosition_msg.data = controller->getPosition();
        motorEncoderPositionPublisher->publish(&motorEncoderPosition_msg);

        motorEncoderVelocity_msg.data = controller->getVelocity();
        motorEncoderVelocityPublisher->publish(&motorEncoderVelocity_msg);

        /**
         * Reduce update frequency for non essential topics
         */
        if (timesUpdated >= 10) {
            motorVelocityControllerStatus_msg = controller->getVelocityControllerStatus();
            motorVelocityControllerStatusPublisher->publish(&motorVelocityControllerStatus_msg);
            timesUpdated = 0;
        }

        timesUpdated++;
        vTaskDelay(waitTime);
    }
}

void MotorTestTask::setpointSubscriberCb(const std_msgs::Float32 &msg) {
    setpointLock->WriterLock();
    targetSetpoint = msg.data;
    setpointLock->WriterUnlock();
}

void MotorTestTask::pidVelocityStateSubscriberCb(const control_msgs::PidState &msg) {
    PIDConfig newConfig = PIDControllerROS::getPIDConfig(msg);
    //if(newConfig != controller->getVelocityControllerConfig()){
    controller->setVelocityControllerConfig(newConfig);
    //}
}

MotorTestTask::~MotorTestTask() {
    delete (setpointLock);
    //delete(setpointSubscriber);
    delete (motorEncoderPositionPublisher);
    delete (motorEncoderVelocityPublisher);
}

/*
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
}*/
