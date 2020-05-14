//
// Created by abiel on 5/12/20.
//

#include "VNH5019.h"

VNH5019::VNH5019(const VNH5019_PinDefinitions &pinDefinitions) {
    this->definitions = pinDefinitions;

    initializePins(pinDefinitions);
}

VNH5019::VNH5019() {
    ;
}

void VNH5019::initializePins(const VNH5019_PinDefinitions &definitions) {
    analogWriteResolution(VNH5019_PWM_BITS);
    analogWriteFrequency(definitions.PWM, VNH5019_PWM_FREQUENCY);

    pinMode(definitions.PWM, OUTPUT);
    pinMode(definitions.IN_A, OUTPUT); pinMode(definitions.IN_B, OUTPUT);
    pinMode(definitions.DIAG_A, INPUT); pinMode(definitions.DIAG_B, INPUT);
}

void VNH5019::set(double value) {
    value = (value < -1.0) ? -1.0 : (1.0 < value) ? 1.0 : value; //Clamp value

    value *= (double) VNH5019_PWM_MAXVAL;
    value = inverted ? -value : value;

    analogWrite(definitions.PWM, std::fabs(value));

    digitalWrite(definitions.IN_A, value >= 0.0 ? HIGH : LOW);
    digitalWrite(definitions.IN_B, value >= 0.0 ? LOW : HIGH);
}