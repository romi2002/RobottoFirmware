//
// Created by abiel on 7/29/20.
//

#ifndef TEENSYROSCONTROLLER_EXPONENTIALODOMETRY_H
#define TEENSYROSCONTROLLER_EXPONENTIALODOMETRY_H

#include "Geometry/Twist2D.h"
#include "Geometry/Pose2D.h"
#include <Eigen.h>

class ExponentialOdometry {
public:
    ExponentialOdometry();

    void update(const Twist2D &position, double yaw);
    void update_velocity(const Twist2D &velocity, double dt, double yaw);

    Pose2D getPose() const;

private:
    Eigen::Matrix<double, 3, 3> rotMatrix;

    void updateRotMatrix(double angle);

    double lastYaw{0};

    Twist2D lastPositionUpdate{0, 0, 0};

    Pose2D currentPose{0, 0, 0};
    Twist2D currentVel{0, 0, 0};
};


#endif //TEENSYROSCONTROLLER_EXPONENTIALODOMETRY_H
