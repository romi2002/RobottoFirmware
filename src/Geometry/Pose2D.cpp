//
// Created by abiel on 7/29/20.
//

#include "Pose2D.h"

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