//
// Created by abiel on 1/28/21.
//

#ifndef TEENSYROSCONTROLLER_WEBSOCKETTASK_H
#define TEENSYROSCONTROLLER_WEBSOCKETTASK_H

#include "config.h"
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include "thread.hpp"
#include <ArduinoWebsockets.h>
#include <TeensyID.h>
#include <ArduinoJson.h>

using namespace cpp_freertos;

class WebSocketTask : public Thread  {
public:
    WebSocketTask(TickType_t tickDelay = 10);

    static DynamicJsonDocument jsonOut;
    static DynamicJsonDocument jsonIn;

    ~WebSocketTask() override{
        ;
    }
private:
    [[noreturn]] void Run() override;

    TickType_t tickDelay;

    websockets::WebsocketsServer server;

    byte mac[6];
    const uint16_t port = 80;
};


#endif //TEENSYROSCONTROLLER_WEBSOCKETTASK_H
