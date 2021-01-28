//
// Created by abiel on 1/27/21.
//

#ifndef TEENSYROSCONTROLLER_NATIVE_HARDWARE_H
#define TEENSYROSCONTROLLER_NATIVE_HARDWARE_H

#include <NativeEthernet.h>

class NATIVE_hardware {
public:
    NATIVE_hardware() {};

    void init() {
        while(!client.connect(server, serverPort))
        {
            Serial.println("Attempting to connect [run serial_node on PC]...");
            delay(1000);
        }
    }

    int read() {
        if (client.available())
        {
            int c = client.read();
            return c;
        }
        return -1;
    }

    void write(uint8_t* data, int length) {
        for (int i = 0; i < length; i++) {
            client.write(data[i]);
        }
    }

    unsigned long time() {
        return millis();
    }
};

#endif //TEENSYROSCONTROLLER_NATIVE_HARDWARE_H
