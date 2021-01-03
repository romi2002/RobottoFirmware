//
// Created by abiel on 7/28/20.
//

#include "Twist2D.h"
#include "Utils/Math/AngleWrap.h"

Twist2D Twist2D::operator-() const {
    return {-dx, -dy, -dtheta};
}

Twist2D Twist2D::operator+(const Twist2D &other) const {
    return {
            dx + other.dx,
            dy + other.dy,
            wrapAngle(dtheta + other.dtheta)
    };
}

Twist2D Twist2D::operator+=(const Twist2D &other) {
    dx += other.dx;
    dy += other.dy;
    dtheta += other.dtheta;
    dtheta = wrapAngle(dtheta);

    return *this;
}

Twist2D &Twist2D::operator-=(const Twist2D &other) {
    *this += -other;
    return *this;
}

Twist2D Twist2D::operator-(const Twist2D &other) const {
    return *this + -other;
}

Twist2D Twist2D::operator*(double rhs) const {
    return{
        this->dx * rhs,
        this->dy * rhs,
        this->dtheta * rhs
    };
}

Twist2D Twist2D::operator/(double rhs) const {
    return{
            this->dx / rhs,
            this->dy / rhs,
            this->dtheta / rhs
    };
}