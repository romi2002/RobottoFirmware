//
// Created by abiel on 6/8/20.
//

#include "PIDController.h"
#include <cmath>

PIDController::PIDController(const PIDConfig &config) {
    this->config = config;
    deltaTime = 0;
}

void PIDController::setSetpoint(double newSetpoint) {
    this->targetSetpoint = newSetpoint;
}

double PIDController::getSetpoint() const {
    return setpoint;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    deltaTime = 0;
}

void PIDController::setConfig(PIDConfig &newConfig) {
    this->config = newConfig;
}

double PIDController::calculate(double sensorVal) {
    const double deltaTimeSeconds = deltaTime / 1e+6;

    if(config.enableRampRate and setpoint != targetSetpoint){
        const double targetDelta = targetSetpoint - setpoint;
        double setpointDelta = std::copysign(config.rampRate * deltaTimeSeconds, targetDelta);
        if(std::fabs(setpointDelta ) > std::fabs(targetDelta)) setpointDelta = targetDelta;
        setpoint += setpointDelta;
    } else {
        setpoint = targetSetpoint;
    }

    const double error = setpoint - sensorVal;

    integral += error * deltaTimeSeconds;
    const double derivative = (error - lastError) / deltaTimeSeconds;

    double output = config.p * error + config.i * integral + config.d * derivative;

    if(fabs(output) < config.deadband){
            output = 0;
        }


    //if(fabs(output) < config.deadband){
    //    output = 0;
    //}

    lastOutput = output;
    lastError = error;
    deltaTime = 0;

    return output;
}