//
// Created by abiel on 5/12/20.
//

#include "VNH5019.h"
#include <cassert>
#include "Adafruit_MCP23017.h"

VNH5019::VNH5019(const VNH5019_PinAssignments &pinDefinitions, Adafruit_MCP23017 *mcp,
                 cpp_freertos::ReadWriteLockPreferWriter *i2cLock) {
    this->definitions = pinDefinitions;

    assert(mcp != nullptr or i2cLock != nullptr);

    this->mcp = mcp;
    this->mcpLock = i2cLock;

    initializePins(pinDefinitions);
}

VNH5019::VNH5019() {
    ;
}

void VNH5019::initializePins(const VNH5019_PinAssignments &definitions) {
    analogWriteResolution(VNH5019_PWM_BITS);
    analogWriteFrequency(definitions.PWM, VNH5019_PWM_FREQUENCY);

    pinMode(definitions.PWM, OUTPUT);
    mcpLock->WriterLock();
    mcp->pinMode(definitions.IN_A, OUTPUT);
    mcp->pinMode(definitions.IN_B, OUTPUT);
    mcp->pinMode(definitions.DIAG_A, INPUT);
    mcp->pinMode(definitions.DIAG_B, INPUT);
    mcpLock->WriterUnlock();
}

void VNH5019::set(double value) {
    if (std::fabs(value) < 0.05) value = 0;
    double tempValue = (value < -1.0) ? -1.0 : (1.0 < value) ? 1.0 : value; //Clamp value

    tempValue = tempValue * (double) VNH5019_PWM_MAXVAL;
    tempValue = inverted ? -tempValue : tempValue;

    analogWrite(definitions.PWM, (int) std::abs(tempValue));
    Serial.println("VH50: Por entrar al lock");
    Serial.println((int)mcpLock);

    mcpLock->WriterLock();
    Serial.println("Nunca entra");
    Serial.println(__LINE__);
    Serial.println((int)mcpLock);

    Serial.println((int) mcp);
    Serial.println(value);

    if (value > 0.0) {
        mcp->digitalWrite(definitions.IN_A, HIGH);
        mcp->digitalWrite(definitions.IN_B, LOW);
    } else if (value < 0.0) {
        mcp->digitalWrite(definitions.IN_A, LOW);
        mcp->digitalWrite(definitions.IN_B, HIGH);
    } else if (value == 0.0) {
        Serial.println("This if");
        mcp->digitalWrite(definitions.IN_A, LOW);
        Serial.println("This line");
        mcp->digitalWrite(definitions.IN_B, LOW);
    }

    Serial.println("Despues del if");
    mcpLock->WriterUnlock();
    Serial.println("VH50: Fuera de set");
}