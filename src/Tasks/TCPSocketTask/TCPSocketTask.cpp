//
// Created by abiel on 1/28/21.
//

#include "TCPSocketTask.h"
#include <NativeEthernet.h>
#include "cobs.h"

OutData TCPSocketTask::dataOut;
InData TCPSocketTask::dataIn;

TCPSocketTask::TCPSocketTask(TickType_t tickDelay) : Thread("HeartbeatTask", 8000,
                                                            2), server(port) {
    teensyMAC(mac);

    while (!Ethernet.begin(mac)) {
        SerialUSB1.println("Connection failed!");
        delay(1000);
    }

    SerialUSB1.print("Connected to: ");
    SerialUSB1.println(Ethernet.localIP());

    server.begin();

    this->tickDelay = tickDelay;
    Start();
}

void TCPSocketTask::Run() {
    TCPWriter writer(&server);
    EthernetClient client;
    uint8_t buffer[4096 * 2];
    uint8_t cobsBuffer[4096 * 2];
    uint8_t readBuffer[4096 * 2];
    uint8_t cobsDenc[4096 * 2];

    size_t readSize = 0;
    size_t readIndex = 0;
    bool newBytes = false;

    for (;;) {
        if (client and client.connected()) {
            dataOut.hb = xTaskGetTickCount();

            /**
             * Out TCP
             */
            jsonOut.clear();
            serializePayload(jsonOut, dataOut);

            int size = measureJson(jsonOut);
            buffer[0] = size & 0xff;
            buffer[1] = (size >> 8) & 0xff;
            size = 2 + serializeJson(jsonOut, buffer + 2, sizeof(buffer) - 2);
            auto res = cobs_encode(cobsBuffer, sizeof(cobsBuffer), buffer, size);

            if (res.status == COBS_ENCODE_OK) {
                cobsBuffer[res.out_len++] = 0x00;
                server.write(cobsBuffer, res.out_len);
            }

            /**
             * In TCP
             */
            while (client.peek() != -1) {
                readBuffer[readSize++] = client.read();
                newBytes = true;
            }

            if(newBytes){
                size_t n = readSize + readIndex;

                for (int i = 0; i < readSize; ++i) {
                    if (readBuffer[i] == 0x00) {
                        //Found end of packet
                        auto res = cobs_decode(cobsDenc, sizeof(cobsDenc), readBuffer, i);
                        if (res.status == COBS_DECODE_OK) {
                            auto ret = deserializeJson(jsonIn, cobsDenc, res.out_len);
                            memmove(readBuffer, readBuffer + i, sizeof(readBuffer) - i);
                            readSize = 0;

                            if(ret.code() == ArduinoJson6172_91::DeserializationError::Ok){
                                deserializePayload(dataIn, jsonIn);
                                SerialUSB1.println(dataIn.i);
                            } else {
                                SerialUSB1.println("json error");
                            }
                            break;
                        }

                        memmove(readBuffer, readBuffer + i + 1, (n + readIndex) - 1 - i);
                        readIndex = n - i - 1;
                        n = n - i - 1;
                        i = 0;
                    }
                }
            }
        } else {
            client = server.available();
        }
        vTaskDelay(tickDelay);
    }
}