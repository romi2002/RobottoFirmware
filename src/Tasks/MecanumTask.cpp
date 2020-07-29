//
// Created by abiel on 7/28/20.
//

#include "MecanumTask.h"
#include "PinAssignments.h"

MecanumTask::MecanumTask(const MotorControllerConfig &config, ros::NodeHandle *nh, TickType_t waitTime) : cpp_freertos::Thread("MecanumTask", 256, MECANUM_TASK_PRIORITY) {
    m_config = config;
    this->waitTime = waitTime;
    this->nh = nh;

    twistLock = new cpp_freertos::ReadWriteLockPreferWriter();

    m_config.encoderPinDefinitions = PinAssignments::getMotor1Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor1Driver();

    controllers.frontLeft = new MotorController("FrontLeft", m_config);

    m_config.encoderPinDefinitions = PinAssignments::getMotor2Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor2Driver();

    controllers.frontRight = new MotorController("FrontRight", m_config);

    m_config.encoderPinDefinitions = PinAssignments::getMotor3Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor3Driver();

    controllers.backLeft = new MotorController("BackLeft", m_config);

    m_config.encoderPinDefinitions = PinAssignments::getMotor4Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor4Driver();

    controllers.backRight = new MotorController("BackRight", m_config);

    MecanumWheelValues<Translation2D> wheelPositions;
    wheelPositions.frontLeft = Translation2D(0.5, 0.5);
    wheelPositions.frontRight = Translation2D(0.5, -0.5);
    wheelPositions.backLeft = Translation2D(-0.5, 0.5);
    wheelPositions.backRight = Translation2D(-0.5, -0.5);

    kinematics = new MecanumKinematics(wheelPositions);

    this->twistSetpointSubscriber = new ros::Subscriber<geometry_msgs::Twist, MecanumTask>(
            "cmd_vel", &MecanumTask::twistSubsriberCb, this
            );

    nh->subscribe(*twistSetpointSubscriber);

    Start();
}

void MecanumTask::writeToMotors(const MecanumWheelVelocities &vel, MotorControlMode controlMode) {
    controllers.frontLeft->set(vel.frontLeft, controlMode);
    controllers.frontRight->set(vel.frontRight, controlMode);
    controllers.backLeft->set(vel.backLeft, controlMode);
    controllers.backRight->set(vel.backRight, controlMode);
}

[[noreturn]] void MecanumTask::Run() {
    while (true){
        twistLock->ReaderLock();
        auto vel = kinematics->toWheelSpeeds(currentTarget);
        twistLock->ReaderUnlock();

        writeToMotors(vel, MotorControlMode::PERCENTAGE);

        vTaskDelay(waitTime);
    }
}

void MecanumTask::twistSubsriberCb(const geometry_msgs::Twist &msg) {
    twistLock->WriterLock();
    currentTarget = Twist2D(msg.linear.x, msg.linear.y, msg.angular.z);
    twistLock->WriterUnlock();
}