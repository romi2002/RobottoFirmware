#pragma once

#include <Arduino.h>
#include "HAL/VNH5019_PinAssignments.h"
#include "HAL/EncoderPinAssignments.h"

namespace PinAssignments{
    const uint8_t HEARTBEAT_PIN = 13;
    const uint8_t  DEBUG_PIN = 12;
    const uint8_t CURRENT_SENSOR_PIN = PIN_A0;
    const uint8_t BATTERY_VOLTAGE_PIN = PIN_A9;

    VNH5019_PinAssignments getMotor1Driver();
    VNH5019_PinAssignments getMotor2Driver();
    VNH5019_PinAssignments getMotor3Driver();
    VNH5019_PinAssignments getMotor4Driver();

    EncoderPinAssignments getMotor1Encoder();
    EncoderPinAssignments getMotor2Encoder();
    EncoderPinAssignments getMotor3Encoder();
    EncoderPinAssignments getMotor4Encoder();
}