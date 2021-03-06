//
// Created by abiel on 6/21/20.
//

#include "PinAssignments.h"

VNH5019_PinAssignments PinAssignments::getMotor1Driver() {
    VNH5019_PinAssignments def{};

    def.PWM = 6;
    def.IN_A = 9;
    def.IN_B = 8;
    def.DIAG_A = 0;
    def.DIAG_B = 1;

    return def;
}

VNH5019_PinAssignments PinAssignments::getMotor2Driver() {
    VNH5019_PinAssignments def{};

    def.PWM = 8;
    def.IN_A = 10;
    def.IN_B = 11;
    def.DIAG_A = 2;
    def.DIAG_B = 3;

    return def;
}

VNH5019_PinAssignments PinAssignments::getMotor3Driver() {
    VNH5019_PinAssignments def{};

    def.PWM = 9;
    def.IN_A = 13;
    def.IN_B = 12;
    def.DIAG_A = 4;
    def.DIAG_B = 5;

    return def;
}

VNH5019_PinAssignments PinAssignments::getMotor4Driver() {
    VNH5019_PinAssignments def{};

    def.PWM = 10;
    def.IN_A = 14;
    def.IN_B = 15;
    def.DIAG_A = 6;
    def.DIAG_B = 7;

    return def;
}

EncoderPinAssignments PinAssignments::getMotor1Encoder() {
    EncoderPinAssignments def{};
    def.channel = 1;
    def.phaseA = 31;
    def.phaseB = 33;

    return def;
}

EncoderPinAssignments PinAssignments::getMotor2Encoder() {
    EncoderPinAssignments def{};
    def.channel = 2;
    def.phaseA = 2;
    def.phaseB = 3;

    return def;
}

EncoderPinAssignments PinAssignments::getMotor3Encoder() {
    EncoderPinAssignments def{};
    def.channel = 3;
    def.phaseA = 5;
    def.phaseB = 4;

    return def;
}

EncoderPinAssignments PinAssignments::getMotor4Encoder() {
    EncoderPinAssignments def{};
    def.channel = 4;
    def.phaseA = 7;
    def.phaseB = 30;

    return def;
}