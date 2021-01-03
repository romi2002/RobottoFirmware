//
// Created by abiel on 7/29/20.
//

#include "Pose2D.h"
#include <cmath>
#include "Rotation2D.h"
#include <Eigen.h>
#include "Utils/Math/AngleWrap.h"

Pose2D Pose2D::operator+(const Pose2D &other) const {
    return {
            x + other.x,
            y + other.y,
            theta + other.theta
    };
}

Pose2D Pose2D::operator+=(const Pose2D &other) {
    x += other.x;
    y += other.y;
    theta += other.theta;

    return *this;
}

Pose2D Pose2D::exp(const Twist2D &delta) {
    double sin_theta = sin(delta.dtheta);
    double cos_theta = cos(delta.dtheta);

    double s, c;
    if (std::abs(delta.dtheta) < 1e-9) {
        s = 1.0 - 1.0 / 6.0 * std::pow(delta.dtheta, 2.0);
        c = 0.5 * delta.dtheta;
    } else {
        s = sin_theta / delta.dtheta;
        c = (1.0 - cos_theta) / delta.dtheta;
    }

    return {delta.dx * s - delta.dy * c,
            delta.dx * c + delta.dy * s,
            Rotation2D(cos_theta, sin_theta).rad()};
}

Pose2D Pose2D::rotateBy(const Rotation2D &rot) const {
    Eigen::Matrix<double, 3, 3> rotMatrix;
    rotMatrix << rot.cos(), -rot.sin(), 0,
                rot.sin(), rot.cos(), 0,
                0, 0, 1;

    Eigen::Matrix<double, 3, 1> posMatrix;
    posMatrix << x, y, theta;

    posMatrix = rotMatrix * posMatrix;
    return {posMatrix(0,0), posMatrix(1,0), posMatrix(2,0)};
}

Pose2D Pose2D::transformBy(const Pose2D &other) const {
    Pose2D transformed = other.rotateBy(Rotation2D(theta));
    transformed += *this;
    transformed.theta = Rotation2D(theta).rotateBy(Rotation2D(other.theta)).rad();

    return transformed;
}