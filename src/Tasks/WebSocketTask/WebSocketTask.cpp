//
// Created by abiel on 1/28/21.
//

#include "WebSocketTask.h"
#include <NativeEthernet.h>

DynamicJsonDocument WebSocketTask::jsonOut(4096);
DynamicJsonDocument WebSocketTask::jsonIn(4096);

WebSocketTask::WebSocketTask(TickType_t tickDelay) : Thread("HeartbeatTask", 8000,
                                                            2) {
    teensyMAC(mac);

    while(!Ethernet.begin(mac)){
        SerialUSB1.println("Connection failed!");
        delay(1000);
    }

    SerialUSB1.print("Connected to: "); SerialUSB1.println(Ethernet.localIP());

    server.listen(port);

    this->tickDelay = tickDelay;

    Start();
}

void WebSocketTask::Run() {
    websockets::WebsocketsClient client = server.accept();
    char buffer[4096];

    int i = 0;

    for(;;){
        if(client.available()){
                client.poll();
              //auto msg = client.readNonBlocking();
//            if(!msg.isEmpty()){
//                SerialUSB1.println("Got msg from client!");
//            }

            jsonOut["hb"] = xTaskGetTickCount();
            auto size = serializeJson(jsonOut, buffer, sizeof(buffer));
            client.send(buffer, size);
        } else {
            client = server.accept();
            client.onMessage([](websockets::WebsocketsClient &client, websockets::WebsocketsMessage msg){
                SerialUSB1.println("got");
                SerialUSB1.println(msg.c_str());
            });
        }
        vTaskDelay(tickDelay);
    }
}