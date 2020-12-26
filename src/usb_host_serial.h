//
// Created by abiel on 12/24/20.
//

#ifndef TEENSYROSCONTROLLER_USB_HOST_SERIAL_H
#define TEENSYROSCONTROLLER_USB_HOST_SERIAL_H

#include <Arduino.h>
#include <USBHost_t36.h>

class usb_host_serial{
public:
    static usb_host_serial& getInstance(){
        static usb_host_serial instance;
        return instance;
    }

    USBHost usbHost;
    USBSerial_BigBuffer userial;

private:
    usb_host_serial() : userial(usbHost, 1){
        USBHost::begin();
        SerialUSB.begin(115200);


        USBHost::Task();
        userial.begin(115200);
        //usbHost.Task();
        SerialUSB.println("ran ctor");
    }
};

#endif //TEENSYROSCONTROLLER_USB_HOST_SERIAL_H
