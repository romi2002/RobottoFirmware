//
// Created by abiel on 7/28/20.
//

#include "MecanumKinematics.h"

MecanumWheelVelocities MecanumKinematics::toWheelSpeeds(const Twist2D &vel, const Translation2D &centerOfRotation) const {
    if(centerOfRotation != prevCoR){
        MecanumWheelValues<Translation2D> newValues;
        newValues.frontLeft = wheelPositions.frontLeft - centerOfRotation;
        newValues.frontRight = wheelPositions.frontRight - centerOfRotation;
        newValues.backLeft = wheelPositions.backLeft - centerOfRotation;
        newValues.backRight = wheelPositions.backRight - centerOfRotation;

        setInverseKinematics(newValues);

        prevCoR = centerOfRotation;
    }

    Eigen::Vector3d chassisSpeedsVector;
    chassisSpeedsVector << vel.dx, vel.dy, vel.dtheta;

    Eigen::Matrix<double, 4, 1> wheelsMatrix =
            inverseKinematics * chassisSpeedsVector;

    MecanumWheelVelocities wheelSpeeds;
    wheelSpeeds.frontLeft = wheelsMatrix(0, 0);
    wheelSpeeds.frontRight = wheelsMatrix(1, 0);
    wheelSpeeds.backLeft = wheelsMatrix(2, 0);
    wheelSpeeds.backRight = wheelsMatrix(3, 0);
    return wheelSpeeds;
}

Twist2D MecanumKinematics::toChassisSpeeds(const MecanumWheelVelocities &wheelVel) const {
    Eigen::Matrix<double, 4, 1> wheelSpeedsMatrix;
    wheelSpeedsMatrix << wheelVel.frontLeft, wheelVel.frontRight, wheelVel.backLeft, wheelVel.backRight;

    Eigen::Vector3d chassisSpeedsVector =
            forwardKinematics.solve(wheelSpeedsMatrix);

    return {
        chassisSpeedsVector(0),
        chassisSpeedsVector(1),
        chassisSpeedsVector(2)
    };
}

void MecanumKinematics::setInverseKinematics(const MecanumWheelValues<Translation2D> &wheelPositions) const {
    inverseKinematics << 1, -1, (-(wheelPositions.frontLeft.X() + wheelPositions.frontLeft.Y())),
            1,  1, (wheelPositions.frontRight.X() - wheelPositions.frontRight.Y()),
            1,  1, (wheelPositions.backLeft.X() - wheelPositions.backLeft.Y()),
            1, -1, (-(wheelPositions.backRight.X() + wheelPositions.backRight.Y()));
    inverseKinematics /= std::sqrt(2.0);
}