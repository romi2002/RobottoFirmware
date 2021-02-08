//
// Created by abiel on 1/28/21.
//

#ifndef TEENSYROSCONTROLLER_TCPPAYLOAD_H
#define TEENSYROSCONTROLLER_TCPPAYLOAD_H

#include "Geometry/Pose2D.h"
#include "Geometry/Twist2D.h"
#include "Geometry/Quaternion.h"
#include "Kinematics/MecanumKinematics.h"

struct OutData {
    unsigned int hb{0};

    Pose2D pose{};

    Twist2D rawKinematics{};
    Twist2D twist{};

    MecanumWheelVelocities wheelPositions{};
    MecanumWheelVelocities wheelVelocities{};
    MecanumWheelVelocities wheelEffort{};

    Quaternion imuQuat;

    Twist2D angularVel{};
    Twist2D linearAccel{};

    double batteryVoltage{0};
};

struct InData {
    int i{0};
    Twist2D setpoint{};
};

#endif //TEENSYROSCONTROLLER_TCPPAYLOAD_H
