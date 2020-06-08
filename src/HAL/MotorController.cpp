//
// Created by abiel on 6/7/20.
//

#include "MotorController.h"

MotorController::MotorController(const std::string &name, const MotorControllerConfig &config, TickType_t updateTime)
        : Thread(name, 256, MOTOR_DRIVER_TASK_PRIORITY) {
    this->config = config;
    this->updateTime = updateTime;

    setpointLock = new ReadWriteLockPreferReader();

    /**
     * PIDConfig init
     */
    this->velocityPIDConfig = config.velocityPIDConfig;
    this->positionPIDConfig = config.positionPIDConfig;

    pidConfigLock = new ReadWriteLockPreferReader();

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
    currentMode = mode;
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
        setpointLock->ReaderLock();
        //TODO Add position and velocity control
        switch (currentMode) {
            case MotorControlMode::PERCENTAGE:
                motor->set(setpoint);
                break;
            default:
                break;
        }
        setpointLock->ReaderUnlock();

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

        vTaskDelay(updateTime);
    }
}