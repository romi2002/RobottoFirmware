//
// Created by abiel on 6/7/20.
//

#include "MotorController.h"
#include "log.h"

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
                              config.encoderPinDefinitions.phaseB,0);

    encoder->setInitConfig();
    encoder->EncConfig.filterCount = 5;
    encoder->EncConfig.filterSamplePeriod = 255;
    encoder->init();

    averageVelocityLock = new ReadWriteLockPreferWriter();
    averagePositionLock = new ReadWriteLockPreferWriter();

    averageVelocity = 0;
    position = 0;

    motor = new VNH5019(config.vnh5019PinDefinitions, config.mcp, config.i2cLock);

    profilerIt = profiler.initProfiler(name.c_str());

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
    double curr_position = this->position;
    averagePositionLock->ReaderUnlock();

    return curr_position;
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
        deltaTimeSeconds = max(deltaTimeSeconds, 0.0f);
        double encoderVelocity = encoderPositionDelta / deltaTimeSeconds;
        encoderVelocity = encoderVelocity / 979.62 * 60.0;

        averagePositionLock->WriterLock();
        position = encoder->read();
        averagePositionLock->WriterUnlock();

        averageVelocityLock->WriterLock();
        if (isnanf(averageVelocity)) averageVelocity = 0;
        averageVelocity += config.velocityFilterAlpha *
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
                velocityPIDLock->WriterLock();
                if (modeChanged) velocityController->reset();
                velocityController->setSetpoint(setpoint);

                averageVelocityLock->ReaderLock();
                motor->set(velocityController->calculate(averageVelocity));
                averageVelocityLock->ReaderUnlock();
                velocityPIDLock->WriterUnlock();
                break;
            case MotorControlMode::POSITION:
                positionPIDLock->WriterLock();
                if (modeChanged) positionController->reset();
                positionController->setSetpoint(setpoint);

                averagePositionLock->ReaderLock();
                motor->set(positionController->calculate(position));
                averagePositionLock->ReaderUnlock();
                positionPIDLock->WriterUnlock();
                break;
            default:
                break;
        }
        modeChanged = false;
        setpointLock->ReaderUnlock();

        TaskProfiler::updateProfiler(profilerIt);
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
