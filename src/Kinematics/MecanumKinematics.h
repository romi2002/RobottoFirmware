//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_MECANUMKINEMATICS_H
#define TEENSYROSCONTROLLER_MECANUMKINEMATICS_H

#include "Geometry/Translation2D.h"
#include "Geometry/Twist2D.h"
#include <Eigen.h>
#include <QR>

template <typename T>
struct MecanumWheelValues {
    T frontLeft, frontRight;
    T backLeft, backRight;
};

typedef MecanumWheelValues<double> MecanumWheelVelocities;

class MecanumKinematics {
public:
    explicit MecanumKinematics(const MecanumWheelValues<Translation2D> &wheelPositions){
        this->wheelPositions = wheelPositions;
        setInverseKinematics(wheelPositions);
        forwardKinematics = inverseKinematics.householderQr();
    }

    MecanumKinematics(const MecanumKinematics&) = default;

    MecanumWheelVelocities toWheelSpeeds(const Twist2D &vel, const Translation2D &centerOfRotation = Translation2D()) const;

    Twist2D toChassisSpeeds(const MecanumWheelVelocities &wheelVel) const;

private:
    mutable Eigen::Matrix<double, 4, 3> inverseKinematics;
    Eigen::HouseholderQR<Eigen::Matrix<double, 4, 3>> forwardKinematics;

    MecanumWheelValues<Translation2D> wheelPositions;

    mutable Translation2D prevCoR;

    void setInverseKinematics( const MecanumWheelValues<Translation2D> &wheelPositions) const;
};


#endif //TEENSYROSCONTROLLER_MECANUMKINEMATICS_H
