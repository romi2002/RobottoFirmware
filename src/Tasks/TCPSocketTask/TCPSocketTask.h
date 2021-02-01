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

static void serializeData(const Pose2D &pose, JsonObject &obj){
    obj["x"] = pose.x;
    obj["y"] = pose.y;
    obj["theta"] = pose.theta;
}

static void serializeData(const Rotation2D &rot, JsonObject &obj){
    obj["rad"] = rot.rad();
}

static void serializeData(const Translation2D &translation, JsonObject &obj){
    obj["x"] = translation.X();
    obj["y"] = translation.Y();
}

static void serializeData(const Twist2D &twist, JsonObject &obj){
    obj["dx"] = twist.dx;
    obj["dy"] = twist.dy;
    obj["dtheta"] = twist.dtheta;
}

static void serializeData(const MecanumWheelVelocities &velocities, JsonObject &obj){
    obj["frontLeft"] = velocities.frontLeft;
    obj["frontRight"] = velocities.frontRight;
    obj["backLeft"] = velocities.backLeft;
    obj["backRight"] = velocities.backRight;
}

static void serializeData(const Quaternion &quat, JsonObject &obj){
    obj["x"] = quat.x;
    obj["y"] = quat.y;
    obj["z"] = quat.z;
    obj["w"] = quat.w;
}

static void deserializeData(Twist2D & twist, const JsonVariantConst &obj){
    twist.dx = obj["dx"];
    twist.dy = obj["dy"];
    twist.dtheta = obj["dtheta"];
}

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
    TCPSocketTask(TickType_t tickDelay = 1);

    ~TCPSocketTask() override{
        ;
    }

    static OutData dataOut;
    static InData dataIn;

    static void serializePayload(JsonDocument &doc, const OutData &data){
        doc["hb"] = data.hb;

        auto pose_obj = doc.createNestedObject("pose");
        serializeData(data.pose, pose_obj);

        auto twist_obj = doc.createNestedObject("twist");
        serializeData(data.twist, twist_obj);

        auto wheelVelocities_obj = doc.createNestedObject("wheel_vel");
        serializeData(data.wheelVelocities, wheelVelocities_obj);

        auto wheelPositions_obj = doc.createNestedObject("wheel_pos");
        serializeData(data.wheelPositions, wheelPositions_obj);

        auto wheelEffort_obj = doc.createNestedObject("wheel_effort");
        serializeData(data.wheelEffort, wheelEffort_obj);

        auto quat_obj = doc.createNestedObject("imu_quat");
        serializeData(data.imuQuat, quat_obj);
    }

    static void deserializePayload(InData &data, const JsonDocument &doc){
        data.i = doc["i"];
        deserializeData(data.setpoint, doc["setpoint"]);
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
