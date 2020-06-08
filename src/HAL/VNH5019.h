//
// Created by abiel on 5/12/20.
//

#ifndef TEENSYROSCONTROLLER_VNH5019_H
#define TEENSYROSCONTROLLER_VNH5019_H

#include <Arduino.h>
#include <cmath>

const float VNH5019_PWM_FREQUENCY = 4577.64; //16kHz
constexpr uint32_t VNH5019_PWM_BITS = 15;
constexpr uint32_t VNH5019_PWM_MAXVAL = 32757;

struct VNH5019_PinDefinitions{
    uint8_t PWM;
    uint8_t IN_A, IN_B;
    uint8_t CS;

    uint8_t DIAG_A, DIAG_B;
};

class VNH5019 {
public:
    VNH5019(const VNH5019_PinDefinitions &pinDefinitions);
    VNH5019();

    void set(double value);
    double getSetpoint() const;

    void invertDirection(bool value);
    bool isInverted() const;

    double readCurrent() const;
private:
    VNH5019_PinDefinitions definitions;

    static void initializePins(const VNH5019_PinDefinitions &definitions);

    bool inverted = false;
    double setpoint = 0.0;
};


#endif //TEENSYROSCONTROLLER_VNH5019_H
