#include "Wire.h"
#include "ADC128D818.h"
#include "FreeRTOS_TEENSY4.h"

#define CONFIG_REG    0x00
#define CONV_RATE_REG 0x07
#define CHANNEL_DISABLE_REG 0x08
#define ADV_CONFIG_REG 0x0B
#define BUSY_STATUS_REG 0x0C

#define READ_REG_BASE 0x20
#define LIMIT_REG_BASE 0x2A

#define TEMP_REGISTER 0x27

#define START_BIT 0
#define INIT_BIT 7

#define EXT_REF_ENABLE 0
#define MODE_SELECT_1 1
#define MODE_SELECT_2 2

#define BUSY_BIT 0
#define NOT_READY_BIT 1

ADC128D818::ADC128D818(uint8_t address) {
    this->addr = address;

    // enable all channels disabled by default
    disabled_mask = 0;

    ref_v = 2.56f;
    ref_mode = INTERNAL_REF;
    op_mode = SINGLE_ENDED_WITH_TEMP;
    conv_mode = CONTINUOUS;
}

void ADC128D818::setReference(double ref_voltage) {
    ref_v = ref_voltage;
}

void ADC128D818::setReferenceMode(reference_mode_t mode) {
    ref_mode = mode;
}

void ADC128D818::setOperationMode(operation_mode_t mode) {
    op_mode = mode;
}

void ADC128D818::setConversionMode(conv_mode_t mode) {
    conv_mode = mode;
}

void ADC128D818::setDisabledMask(uint8_t disabled_mask) {
    this->disabled_mask = disabled_mask;
}

void ADC128D818::begin() {
    // read busy reg until it returns 0
    setRegisterAddress(BUSY_STATUS_REG);

    while (true) {
        digitalWrite(12, LOW);
        auto reg = readCurrentRegister8(false);
        digitalWrite(12, HIGH);
        if ((reg & 1) == 0) {
            break;
        }

        delay(100);
        digitalWrite(12, HIGH);
        delay(100);
    }

    // program advanced config reg
    setRegister(ADV_CONFIG_REG, ref_mode | (op_mode << 1));

    // program conversion rate reg
    setRegister(CONV_RATE_REG, conv_mode);

    // program enabled channels
    setRegister(CHANNEL_DISABLE_REG, disabled_mask);

    // program limit regs
    // currently noop!

    // set start bit in configuration (interrupts disabled)
    setRegister(CONFIG_REG, 1);
}

uint16_t ADC128D818::read(uint8_t channel, bool useRTOSDelay) {
    setRegisterAddress( READ_REG_BASE + channel);
    Wire.requestFrom(addr, (uint8_t) 2);
    while (!Wire.available()) {
        if(useRTOSDelay){
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            delay(1);
        }
    }
    uint8_t high_byte = Wire.read();
    uint8_t low_byte = Wire.read();

    //SerialUSB1.print("h: ");
    //SerialUSB1.print((((((uint16_t) high_byte) << 8) | ((uint16_t) low_byte)) & 0xFFF0) >> 4);
    //SerialUSB1.print(" l: ");
    //SerialUSB1.print((((((uint16_t) low_byte) << 8) | ((uint16_t) high_byte)) & 0xFFF0) >> 4);


    uint16_t result = (((((uint16_t) high_byte) << 8) | ((uint16_t) low_byte)) & 0xFFF0) >> 4;
    return result;
}

double ADC128D818::readConverted(uint8_t channel, bool useRTOSDelay) {
    return (double) read(channel, useRTOSDelay) / 4096.0 * ref_v;
}

double ADC128D818::readTemperatureConverted(bool useRTOSDelay) {
    setRegisterAddress(READ_REG_BASE + 7);
    Wire.requestFrom(addr, (uint8_t) 2);
    while (!Wire.available()) {
        if(useRTOSDelay){
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            delay(1);
        }
    }
    uint8_t high_byte = Wire.read();
    uint8_t low_byte = Wire.read();

    uint16_t raw = (((((uint16_t) high_byte) << 8) | ((uint16_t) low_byte)) & 0xFF10) >> 7;
    bool msb = raw & 0b100000000u;

    if(msb){
        return ((pow(2.0, 9.0) - (double) raw) / 2.0);
    } else {
        return (double) raw / 2.0;
    }
}


//
// private methods
//

void ADC128D818::setRegisterAddress(uint8_t reg_addr) {
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
}

void ADC128D818::setRegister(uint8_t reg_addr, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg_addr);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t ADC128D818::readCurrentRegister8(bool useRTOSDelay) {
    Wire.requestFrom(addr, (uint8_t) 1, (uint8_t) 1);
    while (!Wire.available()) {
        if(useRTOSDelay){
            vTaskDelay(1);
        } else {
            delay(1);
        }
    }
    return Wire.read();
}
