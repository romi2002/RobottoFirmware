//
// Created by abiel on 7/28/20.
//

#include "MecanumTask.h"
#include "PinAssignments.h"

extern Quaternion imuQuat;
extern double imuYaw;
extern double imuAccel[3], imuAngVel[3];

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

    m_config.encoderPinDefinitions = PinAssignments::getMotor3Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor3Driver();

    controllers.backLeft = new MotorController("BackLeft", m_config);

    m_config.encoderPinDefinitions = PinAssignments::getMotor4Encoder();
    m_config.vnh5019PinDefinitions = PinAssignments::getMotor4Driver();

    controllers.backRight = new MotorController("BackRight", m_config);

    MecanumWheelValues<Translation2D> wheelPositions;
    wheelPositions.frontLeft = Translation2D(0.08, 0.08);
    wheelPositions.frontRight = Translation2D(0.08, -0.08);
    wheelPositions.backLeft = Translation2D(-0.08, 0.08);
    wheelPositions.backRight = Translation2D(-0.08, -0.08);

    kinematics = new MecanumKinematics(wheelPositions);

    odometry = new ExponentialOdometry();

    profilerIt = profiler.initProfiler("MecanumTask");

    lastUpdate = 0;
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
    double initialYaw = imuYaw;

    while (true) {
        twistLock->ReaderLock();
        auto motor_vels = kinematics->toWheelSpeeds(inData.setpoint);
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
        outData.rawKinematics = positions;

        odometry->update_velocity(velocities, (double) lastUpdate / (double) 1e+6f, -imuYaw);

        const auto currentPose = odometry->getPose();

        outData.pose = currentPose;
        outData.twist = velocities;
        outData.imuQuat = imuQuat;

        //TODO
        outData.linearAccel.dx = imuAccel[0];
        outData.linearAccel.dy = imuAccel[1];
        outData.linearAccel.dtheta = imuAccel[2];

        outData.angularVel.dx = imuAngVel[0];
        outData.angularVel.dy = imuAngVel[1];
        outData.angularVel.dtheta = imuAngVel[2];

        writeToMotors(motor_vels, MotorControlMode::PERCENTAGE);

        currentWheelPositions = getWheelPositions();
        currentWheelVelocities = getWheelVelocities();

        outData.wheelPositions = currentWheelPositions;
        outData.wheelVelocities = currentWheelVelocities;
        outData.wheelEffort = motor_vels;

        TaskProfiler::updateProfiler(profilerIt);

        lastUpdate = 0;
        vTaskDelay(waitTime);
    }
}