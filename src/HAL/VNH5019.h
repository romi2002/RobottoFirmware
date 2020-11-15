//
// Created by abiel on 5/12/20.
//

#ifndef TEENSYROSCONTROLLER_VNH5019_H
#define TEENSYROSCONTROLLER_VNH5019_H

#include <Arduino.h>
#include <cmath>
#include <read_write_lock.hpp>
#include "VNH5019_PinAssignments.h"

const float VNH5019_PWM_FREQUENCY = 4577.64; //16kHz
constexpr uint32_t VNH5019_PWM_BITS = 15;
constexpr uint32_t VNH5019_PWM_MAXVAL = 32757;

class Adafruit_MCP23017;

class VNH5019 {
public:
    VNH5019(const VNH5019_PinAssignments &pinDefinitions, Adafruit_MCP23017 *mcp,
            cpp_freertos::ReadWriteLockPreferWriter *i2cLock);

    VNH5019();

    void set(double value);

    double getSetpoint() const;

    void invertDirection(bool value);

    bool isInverted() const;

    double readCurrent() const;

private:
    VNH5019_PinAssignments definitions;

    Adafruit_MCP23017 *mcp;
    cpp_freertos::ReadWriteLockPreferWriter *mcpLock;

    void initializePins(const VNH5019_PinAssignments &definitions);

    bool inverted = false;
    double setpoint = 0.0;
};


#endif //TEENSYROSCONTROLLER_VNH5019_H
