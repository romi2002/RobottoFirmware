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
        while(!client.connect(server_ip, server_port))
        {
            SerialUSB1.println("Attempting to connect [run serial_node on PC]...");
            delay(1000);
        }

        SerialUSB1.println("Connected!");
    }

    int read() {
        SerialUSB1.println("Read");
        if (client.available())
        {
            int c = client.read();
            return c;
        }
        return -1;
    }

    void write(uint8_t* data, int length) {
        SerialUSB1.println("Write");
        client.write(data, length);
    }

    unsigned long time() {
        return millis();
    }
private:
    EthernetClient client;
    IPAddress server_ip{192,168,1,53};
    const uint16_t  server_port = 11411;
};

#endif //TEENSYROSCONTROLLER_NATIVE_HARDWARE_H
