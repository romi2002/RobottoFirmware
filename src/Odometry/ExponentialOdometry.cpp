//
// Created by abiel on 7/29/20.
//

#include "ExponentialOdometry.h"
#include <Eigen.h>

ExponentialOdometry::ExponentialOdometry() {
    updateRotMatrix(0);
}

Pose2D ExponentialOdometry::getPose() const {
    return currentPose;
}

void ExponentialOdometry::updateRotMatrix(double angle) {
    const double angleSin = std::sin(angle);
    const double angleCos = std::cos(angle);

    rotMatrix << angleCos, -angleSin, 0,
            angleSin, angleCos, 0,
            0, 0, 1;
}

void ExponentialOdometry::update(const Twist2D &position, double yaw) {
    Twist2D deltaPos = position - lastPositionUpdate;

    update_velocity(deltaPos, 1, yaw);
    lastPositionUpdate = position;
}

void ExponentialOdometry::update_velocity(const Twist2D &velocity, double dt, double yaw) {
    Twist2D rotVel{-velocity.dx, -velocity.dy, (lastYaw - yaw)};

    currentPose = currentPose.transformBy(Pose2D::exp(rotVel * dt));
    currentPose.theta = yaw;
    lastYaw = yaw;
}