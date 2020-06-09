//
// Created by abiel on 6/8/20.
//

#ifndef TEENSYROSCONTROLLER_PIDCONTROLLER_H
#define TEENSYROSCONTROLLER_PIDCONTROLLER_H

#include "PIDConfig.h"
#include <Arduino.h>

class PIDController {
public:
    explicit PIDController(const PIDConfig &config);

    void setSetpoint(double newSetpoint);
    double getSetpoint() const;

    double calculate(double sensorVal);

    void reset(); //Keeps setpoint

    void setConfig(const PIDConfig &newConfig);

    double getError() const;
    double getErrorDot() const;

    double getP_Error() const;
    double getI_Error() const;
    double getD_Error() const;

    double getLastOutput() const;

    double getLastTimestep() const;

    PIDConfig getCurrentConfig() const;
private:
    PIDConfig config;
    elapsedMicros deltaTime;

    double setpoint{0};
    double targetSetpoint{0};

    double lastOutput{0};
    double lastError{0};
    double errorDot{0};
    double lastDeltaTime{0};

    double integral{0};

    double lastPError{0};
    double lastIError{0};
    double lastDError{0};
};


#endif //TEENSYROSCONTROLLER_PIDCONTROLLER_H
