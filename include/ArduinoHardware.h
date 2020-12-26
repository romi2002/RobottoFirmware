#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <usb_host_serial.h> // Teensy 3.0 and 3.1
#define SERIAL_CLASS usb_serial_class

class ArduinoHardware
{
public:
    ArduinoHardware()
    {
        iostream = &Serial;
        baud_ = 57600;
    }

    void setBaud(long baud)
    {
        this->baud_ = baud;
    }

    int getBaud()
    {
        return baud_;
    }

    void init()
    {
        iostream->begin(baud_);
    }

    int read()
    {
        return iostream->read();
    };

    void write(uint8_t *data, int length)
    {
        for (int i = 0; i < length; i++)
        {
            iostream->write(data[i]);
        }
    }

    unsigned long time()
    {
        return millis();
    }

protected:
    SERIAL_CLASS *iostream;
    long baud_;
};

#endif
