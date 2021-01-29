//
// Created by abiel on 1/28/21.
//

#ifndef TEENSYROSCONTROLLER_TCPSOCKETTASK_H
#define TEENSYROSCONTROLLER_TCPSOCKETTASK_H

#include "config.h"
#include <Arduino.h>
#include <NativeEthernet.h>
#include <FreeRTOS_TEENSY4.h>
#include "thread.hpp"
#include <TeensyID.h>
#include <ArduinoJson.h>
#include "TCPPayload.h"

using namespace cpp_freertos;

class TCPWriter{
public:
    explicit TCPWriter(EthernetServer *server){
        this->server = server;
    }

    size_t write(uint8_t c) {
        if(!server) return 0;
        return server->write(c);
    }

    size_t write(const uint8_t *buffer, size_t length){
        if(!server) return 0;
        return server->write(buffer, length);
    }
private:
    EthernetServer *server;
};

class TCPSocketTask : public Thread  {
public:
    TCPSocketTask(TickType_t tickDelay = 10);

    ~TCPSocketTask() override{
        ;
    }

    static OutData dataOut;
    static InData dataIn;

    static void serializePayload(JsonDocument &doc, const OutData &data){
        doc["hb"] = data.hb;
    }

    static void deserializePayload(InData &data, const JsonDocument &doc){
        data.i = doc["i"];
    }
private:
    [[noreturn]] void Run() override;

    TickType_t tickDelay;

    byte mac[6];
    const uint16_t port = 80;

    DynamicJsonDocument jsonOut{4096};
    DynamicJsonDocument jsonIn{4096};

    EthernetServer server;
};

#endif //TEENSYROSCONTROLLER_TCPSOCKETTASK_H
