//
// Created by abiel on 7/28/20.
//

#include "MecanumTask.h"
#include "PinAssignments.h"

extern geometry_msgs::Quaternion imuQuat;
extern double imuYaw;

MecanumTask::MecanumTask(const MotorControllerConfig &config, ros::NodeHandle *nh, double wheelDiameter,
                         TickType_t waitTime) : cpp_freertos::Thread("MecanumTask", 256, MECANUM_TASK_PRIORITY) {
    m_config = config;
    this->waitTime = waitTime;
    this->nh = nh;
    this->wheelDiameter = wheelDiameter;
    wheelCircumference = M_PI * wheelDiameter;

    twistLock = new cpp_freertos::ReadWriteLockPreferWriter();

    m_config.encoderPinDefinitions = PinAssignments::getMotor1Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor1Driver();

    controllers.frontLeft = new MotorController("FrontLeft", m_config);

    m_config.encoderPinDefinitions = PinAssignments::getMotor2Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor2Driver();

    controllers.frontRight = new MotorController("FrontRight", m_config);

    m_config.encoderPinDefinitions = PinAssignments::getMotor4Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor4Driver();

    controllers.backLeft = new MotorController("BackLeft", m_config);

    m_config.encoderPinDefinitions = PinAssignments::getMotor3Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor3Driver();

    controllers.backRight = new MotorController("BackRight", m_config);

    MecanumWheelValues <Translation2D> wheelPositions;
    wheelPositions.frontLeft = Translation2D(0.08, 0.08);
    wheelPositions.frontRight = Translation2D(0.08, -0.08);
    wheelPositions.backLeft = Translation2D(-0.08, 0.08);
    wheelPositions.backRight = Translation2D(-0.08, -0.08);

    kinematics = new MecanumKinematics(wheelPositions);

    odometry = new ExponentialOdometry();

    this->twistSetpointSubscriber = new ros::Subscriber<geometry_msgs::Twist, MecanumTask>(
            "cmd_vel", &MecanumTask::twistSubsriberCb, this
    );

    twistPublisher = new ros::Publisher("twist", &twistPublisherMsg);
    nh->advertise(*twistPublisher);

    posePublisher = new ros::Publisher("pose", &posePublisherMsg);
    nh->advertise(*posePublisher);

    nh->subscribe(*twistSetpointSubscriber);

    Start();
}

void MecanumTask::writeToMotors(const MecanumWheelVelocities &vel, MotorControlMode controlMode) {
    controllers.frontLeft->set(vel.frontLeft, controlMode);
    controllers.frontRight->set(vel.frontRight, controlMode);
    controllers.backLeft->set(vel.backLeft, controlMode);
    controllers.backRight->set(vel.backRight, controlMode);
}

MecanumWheelVelocities MecanumTask::getWheelPositions() const {
    MecanumWheelVelocities out;
    out.frontLeft = controllers.frontLeft->getPosition() / encoderCodes * (2.0 * M_PI);
    out.frontRight = controllers.frontRight->getPosition() / encoderCodes * (2.0 * M_PI);
    out.backLeft = controllers.backLeft->getPosition() / encoderCodes * (2.0 * M_PI);
    out.backRight = controllers.backRight->getPosition() / encoderCodes * (2.0 * M_PI);

    return out;
}

MecanumWheelVelocities MecanumTask::getWheelVelocities() const {
    MecanumWheelVelocities out;

    out.frontLeft = controllers.frontLeft->getVelocity() / encoderCodes * (2.0 * M_PI);
    out.frontRight = controllers.frontRight->getVelocity() / encoderCodes * (2.0 * M_PI);
    out.backLeft = controllers.backLeft->getVelocity() / encoderCodes * (2.0 * M_PI);
    out.backRight = controllers.backRight->getVelocity() / encoderCodes * (2.0 * M_PI);

    return out;
}

[[noreturn]] void MecanumTask::Run() {
    while (true) {
        twistLock->ReaderLock();
        auto vel = kinematics->toWheelSpeeds(currentTarget);
        twistLock->ReaderUnlock();

        auto currentWheelPositions = getWheelPositions();
        auto currentWheelVelocities = getWheelVelocities();

        currentWheelPositions.frontLeft *= (wheelDiameter / 2.0);
        currentWheelPositions.frontRight *= (wheelDiameter / 2.0);
        currentWheelPositions.backLeft *= (wheelDiameter / 2.0);
        currentWheelPositions.backRight *= (wheelDiameter / 2.0);

        currentWheelVelocities.frontLeft *= (wheelDiameter / 2.0);
        currentWheelVelocities.frontRight *= (wheelDiameter / 2.0);
        currentWheelVelocities.backLeft *= (wheelDiameter / 2.0);
        currentWheelVelocities.backRight *= (wheelDiameter / 2.0);

        auto positions = kinematics->toChassisSpeeds(currentWheelPositions);
        auto velocities = kinematics->toChassisSpeeds(currentWheelVelocities);

        positions.dtheta = imuYaw;

        odometry->update(positions);

        const auto currentPose = odometry->getPose();

        posePublisherMsg.position.x = currentPose.x;
        posePublisherMsg.position.y = currentPose.y;
        posePublisherMsg.position.z = 0;

        twistPublisherMsg.linear.x = velocities.dx;
        twistPublisherMsg.linear.y = velocities.dy;
        twistPublisherMsg.angular.z = velocities.dtheta;

        //TODO Publish orientation from IMU
        posePublisherMsg.orientation = imuQuat;

        posePublisher->publish(&posePublisherMsg);
        twistPublisher->publish(&twistPublisherMsg);

        writeToMotors(vel, MotorControlMode::PERCENTAGE);
        vTaskDelay(waitTime);
    }
}

void MecanumTask::twistSubsriberCb(const geometry_msgs::Twist &msg) {
    twistLock->WriterLock();
    currentTarget = Twist2D(msg.linear.x, msg.linear.y, msg.angular.z);
    twistLock->WriterUnlock();
}