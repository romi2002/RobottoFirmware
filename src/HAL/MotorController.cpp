//
// Created by abiel on 6/7/20.
//

#include "MotorController.h"

MotorController::MotorController(const std::string &name, const MotorControllerConfig &config, TickType_t updateTime)
        : Thread(name, 256, MOTOR_DRIVER_TASK_PRIORITY) {
    this->config = config;
    this->updateTime = updateTime;

    setpointLock = new ReadWriteLockPreferReader();

    velocityController = new PIDController(config.velocityPIDConfig);
    positionController = new PIDController(config.positionPIDConfig);

    velocityPIDLock = new ReadWriteLockPreferReader();
    positionPIDLock = new ReadWriteLockPreferReader();

    encoder = new QuadEncoder(config.encoderPinDefinitions.channel, config.encoderPinDefinitions.phaseA,
                              config.encoderPinDefinitions.phaseB);

    encoder->setInitConfig();
    encoder->EncConfig.filterCount = 5;
    encoder->EncConfig.filterSamplePeriod = 255;
    encoder->init();

    averageVelocityLock = new ReadWriteLockPreferWriter();
    averagePositionLock = new ReadWriteLockPreferWriter();

    motor = new VNH5019(config.vnh5019PinDefinitions);

    Start();
}

void MotorController::set(double setpoint, MotorControlMode mode) {
    setpointLock->WriterLock();
    this->setpoint = setpoint;
    if (currentMode != mode) {
        modeChanged = true;
        currentMode = mode;
    }
    setpointLock->WriterUnlock();
}

double MotorController::getPosition() const {
    averagePositionLock->ReaderLock();
    double position = averagePosition;
    averagePositionLock->ReaderUnlock();

    return position;
}

double MotorController::getVelocity() const {
    averageVelocityLock->ReaderLock();
    double velocity = averageVelocity;
    averageVelocityLock->ReaderUnlock();

    return velocity;
}

[[noreturn]] void MotorController::Run() {
    elapsedMicros deltaTime;
    int32_t lastPosition = encoder->read();

    while (true) {
        const int32_t encoderPosition = encoder->read();

        auto encoderPositionDelta = static_cast<float>(encoderPosition - lastPosition);
        auto deltaTimeSeconds = static_cast<float>(deltaTime) / 1e+6f;
        float encoderVelocity = encoderPositionDelta / deltaTimeSeconds;
        encoderVelocity = encoderVelocity / 979.62 * 60.0;

        averagePositionLock->WriterLock();
        averagePosition = averagePosition + config.positionFilterAlpha * (encoderPosition - averagePosition);
        averagePositionLock->WriterUnlock();

        averageVelocityLock->WriterLock();
        averageVelocity = averageVelocity + config.velocityFilterAlpha *
                                            (encoderVelocity - averageVelocity); //Exponential rolling average
        averageVelocityLock->WriterUnlock();

        lastPosition = encoder->read();
        deltaTime = 0;

        setpointLock->ReaderLock();
        //TODO Add position and velocity control
        switch (currentMode) {
            case MotorControlMode::PERCENTAGE:
                motor->set(setpoint);
                break;
            case MotorControlMode::VELOCITY:
                if (modeChanged) velocityController->reset();
                velocityPIDLock->WriterLock();
                velocityController->setSetpoint(setpoint);

                averageVelocityLock->ReaderLock();
                motor->set(velocityController->calculate(averageVelocity));
                averageVelocityLock->ReaderUnlock();
                velocityPIDLock->WriterUnlock();
                break;
            default:
                break;
        }
        modeChanged = false;
        setpointLock->ReaderUnlock();

        vTaskDelay(updateTime);
    }
}

MotorController::~MotorController() {
    delete (setpointLock);
    delete (velocityController);
    delete (positionController);
    delete (velocityPIDLock);
    delete (positionPIDLock);
    delete (encoder);
    delete (averageVelocityLock);
    delete (averagePositionLock);
    delete (motor);
}

control_msgs::PidState MotorController::getVelPIDStatus() const {
    control_msgs::PidState msg;
    velocityPIDLock->ReaderLock();
    msg = PIDControllerROS::getPIDState(velocityController);
    velocityPIDLock->ReaderUnlock();

    return msg;
}

control_msgs::PidState MotorController::getPosPIDStatus() const {
    control_msgs::PidState msg;
    positionPIDLock->ReaderLock();
    msg = PIDControllerROS::getPIDState(positionController);
    positionPIDLock->ReaderUnlock();

    return msg;
}

PIDConfig MotorController::getVelPIDConfig() const {
    PIDConfig configOut;
    positionPIDLock->ReaderLock();
    configOut = positionController->getCurrentConfig();
    positionPIDLock->ReaderUnlock();
    return configOut;
}

void MotorController::setVelPIDConfig(const PIDConfig &configIn) {
    velocityPIDLock->WriterLock();
    velocityController->setConfig(configIn);
    velocityPIDLock->WriterUnlock();
}

PIDConfig MotorController::getPosPIDConfig() const {
    PIDConfig configOut;
    velocityPIDLock->ReaderLock();
    configOut = velocityController->getCurrentConfig();
    velocityPIDLock->ReaderUnlock();
    return configOut;
}

void MotorController::setPosPIDConfig(const PIDConfig &configIn) {
    positionPIDLock->WriterLock();
    positionController->setConfig(configIn);
    positionPIDLock->WriterUnlock();
}