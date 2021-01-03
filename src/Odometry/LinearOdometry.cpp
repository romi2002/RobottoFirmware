//
// Created by abiel on 1/1/21.
//

#include "LinearOdometry.h"
#include <cmath>

LinearOdometry::LinearOdometry() {
    ;
}

void LinearOdometry::update(const Twist2D &newPosition, double yaw) {
    auto deltaPos = newPosition - lastPos;

    auto angleCos = std::cos(yaw);
    auto angleSin = std::sin(yaw);

    double rotX = deltaPos.dx * angleCos - deltaPos.dy * angleSin;
    double rotY = deltaPos.dx * angleSin + deltaPos.dy * angleCos;

    currentPose += Pose2D(-rotX, -rotY, 0);
    currentPose.theta = yaw;

    lastPos = newPosition;
}

Pose2D LinearOdometry::getPose() const {
    return currentPose;
}