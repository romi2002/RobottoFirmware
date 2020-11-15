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

void ExponentialOdometry::update(const Twist2D &position, double startAngle) {
    Twist2D deltaPos = position - lastPositionUpdate;

    if (lastStartAngle != startAngle) {
        updateRotMatrix(startAngle);
        lastStartAngle = startAngle;
    }

    const double deltaSin = std::sin(deltaPos.dtheta);
    const double deltaCos = std::cos(deltaPos.dtheta);

    Eigen::Matrix<double, 3, 1> deltaMatrix;
    deltaMatrix << deltaPos.dx, deltaPos.dy, deltaPos.dtheta;

    Eigen::Matrix<double, 3, 1> finalMatrix;
    if (deltaPos.dtheta != 0) {
        Eigen::Matrix<double, 3, 3> velMatrix;
        velMatrix << deltaSin / deltaPos.dtheta, (deltaCos - 1.0) / deltaPos.dtheta, 0,
                (1.0 - deltaCos) / deltaPos.dtheta, deltaSin / deltaPos.dtheta, 0,
                0, 0, 1;
        finalMatrix = rotMatrix * velMatrix * deltaMatrix;
    } else {
        finalMatrix = rotMatrix * deltaMatrix;
    }

    Pose2D posUpdate(
            finalMatrix(0, 0),
            finalMatrix(1, 0),
            finalMatrix(2, 0)
    );

    currentPose += posUpdate;

    lastPositionUpdate = position;
}