//
// Created by abiel on 6/8/20.
//

#ifndef TEENSYROSCONTROLLER_PIDCONTROLLER_H
#define TEENSYROSCONTROLLER_PIDCONTROLLER_H

#include "PIDConfig.h"
#include <Arduino.h>

class PIDController {
public:
    PIDController(const PIDConfig &config);

    void setSetpoint(double newSetpoint);
    double getSetpoint() const;

    double calculate(double sensorVal);

    void reset(); //Keeps setpoint

    void setConfig(PIDConfig &newConfig);
private:
    PIDConfig config;
    elapsedMicros deltaTime;

    double setpoint{0};
    double targetSetpoint{0};

    double lastOutput{0};
    double lastError{0};
    double integral{0};
};


#endif //TEENSYROSCONTROLLER_PIDCONTROLLER_H
