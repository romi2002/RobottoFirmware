//
// Created by abiel on 5/12/20.
//

#include <functional>
#include "MotorTestTask.h"

MotorTestTask::MotorTestTask(const std::string &name, ros::NodeHandle *nh, VNH5019 *motor, uint8_t encoderChannel, uint8_t phaseA, uint8_t phaseB, TickType_t waitTime) : Thread(name, 256, MOTOR_OUTPUT_TEST_TASK_PRIORITY) {
    this->name = name;
    this->nh = nh;
    this->motor = motor;
    this->quadEncoder = new QuadEncoder(encoderChannel, phaseA, phaseB);

    setpointTopic = name + "_setpoint";
    encoderPositionTopic = name + "_position";
    encoderVelocityTopic = name + "_velocity";

    quadEncoder->setInitConfig();
    quadEncoder->EncConfig.filterCount = 5;
    quadEncoder->EncConfig.filterSamplePeriod = 255;
    //quadEncoder->EncConfig.INDEXTriggerMode = RISING_EDGE;

    quadEncoder->init();

    this->waitTime = waitTime;

    this->setpointLock = new ReadWriteLockPreferReader();

    this->setpointSubscriber = new ros::Subscriber<std_msgs::Float32, MotorTestTask>(setpointTopic.data(), &MotorTestTask::setpointSubscriberCb, this, 1);

    nh->subscribe(*setpointSubscriber);

    motorEncoderPositionPublisher = new ros::Publisher(encoderPositionTopic.data(), &motorEncoderPosition_msg);
    motorEncoderVelocityPublisher = new ros::Publisher(encoderVelocityTopic.data(), &motorEncoderVelocity_msg);

    nh->advertise(*motorEncoderPositionPublisher);
    nh->advertise(*motorEncoderVelocityPublisher);

    Start();
}

[[noreturn]] void MotorTestTask::Run() {
    //auto lastTime = xTaskGetTickCount();
    elapsedMicros deltaTime;
    int32_t lastPosition = quadEncoder->read();
    float currentVelocityAverage = 0;
    float currentPositionAverage = 0;

    while(true){
        setpointLock->ReaderLock();
        motor->set(targetSetpoint);
        setpointLock->ReaderUnlock();

        const int32_t encoderPosition = quadEncoder->read();

        auto encoderPositionDelta = static_cast<float>(encoderPosition - lastPosition);
        auto deltaTimeSeconds = static_cast<float>(deltaTime) / 1e+6f;
        float encoderVelocity = encoderPositionDelta / deltaTimeSeconds;
        encoderVelocity = encoderVelocity / 979.62 * 60.0;

        currentPositionAverage = currentPositionAverage + positionAverageAlpha * (encoderPosition - currentPositionAverage);
        currentVelocityAverage = currentVelocityAverage + velocityAverageAlpha * (encoderVelocity - currentVelocityAverage); //Exponential rolling average

        motorEncoderPosition_msg.data = currentPositionAverage;
        motorEncoderPositionPublisher->publish(&motorEncoderPosition_msg);

        motorEncoderVelocity_msg.data = currentVelocityAverage;
        motorEncoderVelocityPublisher->publish(&motorEncoderVelocity_msg);


        lastPosition = encoderPosition;
        //lastTime = xTaskGetTickCount();
        deltaTime = 0;

        vTaskDelay(waitTime);
    }
}

void MotorTestTask::setpointSubscriberCb(const std_msgs::Float32 &msg) {
    setpointLock->WriterLock();
    targetSetpoint = msg.data;
    setpointLock->WriterUnlock();
}

MotorTestTask::~MotorTestTask() {
    delete(quadEncoder);
    delete(setpointLock);
    //delete(setpointSubscriber);
    delete(motorEncoderPositionPublisher);
    delete(motorEncoderVelocityPublisher);
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
