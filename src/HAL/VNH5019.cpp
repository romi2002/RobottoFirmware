//
// Created by abiel on 5/12/20.
//

#include "VNH5019.h"

VNH5019::VNH5019(const VNH5019_PinAssignments &pinDefinitions) {
    this->definitions = pinDefinitions;

    initializePins(pinDefinitions);
}

VNH5019::VNH5019() {
    ;
}

void VNH5019::initializePins(const VNH5019_PinAssignments &definitions) {
    analogWriteResolution(VNH5019_PWM_BITS);
    analogWriteFrequency(definitions.PWM, VNH5019_PWM_FREQUENCY);

    pinMode(definitions.PWM, OUTPUT);
    pinMode(definitions.IN_A, OUTPUT);
    pinMode(definitions.IN_B, OUTPUT);
    pinMode(definitions.DIAG_A, INPUT);
    pinMode(definitions.DIAG_B, INPUT);
}

void VNH5019::set(double value) {
    double tempValue = (value < -1.0) ? -1.0 : (1.0 < value) ? 1.0 : value; //Clamp value

    tempValue = tempValue * (double) VNH5019_PWM_MAXVAL;
    tempValue = inverted ? -tempValue : tempValue;

    analogWrite(definitions.PWM, (int) std::abs(tempValue));

    if (value > 0.0) {
        digitalWriteFast(definitions.IN_A, HIGH);
        digitalWriteFast(definitions.IN_B, LOW);
    } else if (value < 0.0) {
        digitalWriteFast(definitions.IN_A, LOW);
        digitalWriteFast(definitions.IN_B, HIGH);
    } else if (value == 0.0) {
        digitalWriteFast(definitions.IN_A, LOW);
        digitalWriteFast(definitions.IN_B, LOW);
    }
}