//
// Created by abiel on 7/28/20.
//

#include <cmath>
#include "Translation2D.h"

Translation2D::Translation2D(double x, double y) {
    this->x = x;
    this->y = y;
}

double Translation2D::distance(const Translation2D &rhs) const {
    return std::hypot(rhs.x - x, rhs.y - y);
}

double Translation2D::norm() const {
    return std::hypot(x, y);
}

Translation2D Translation2D::rotateBy(const Rotation2D &other) const {
    return {
        x * other.cos() - y * other.sin(),
        x * other.sin() + y * other.cos()
    };
}

Translation2D Translation2D::operator+(const Translation2D &other) const {
    return {X() + other.X(), Y() + other.Y()};
}

Translation2D& Translation2D::operator+=(const Translation2D& other) {
    x += other.X();
    y += other.Y();
    return *this;
}

Translation2D Translation2D::operator-(const Translation2D& other) const {
    return *this + -other;
}

Translation2D& Translation2D::operator-=(const Translation2D& other) {
    *this += -other;
    return *this;
}

Translation2D Translation2D::operator-() const { return {-x, -y}; }

Translation2D Translation2D::operator*(double scalar) const {
    return {scalar * x, scalar * y};
}

Translation2D& Translation2D::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

Translation2D Translation2D::operator/(double scalar) const {
    return *this * (1.0 / scalar);
}

Translation2D& Translation2D::operator/=(double scalar) {
    *this *= (1.0 / scalar);
    return *this;
}

bool Translation2D::operator==(const Translation2D &rhs) const {
    return std::abs(x - rhs.X()) < 1E-9 &&
            std::abs(y - rhs.Y()) < 1E-9;
}

bool Translation2D::operator!=(const Translation2D &rhs) const {
    return !(rhs == *this);
}

double Translation2D::X() const {
    return x;
}

void Translation2D::setX(double x) {
    Translation2D::x = x;
}

double Translation2D::Y() const {
    return y;
}

void Translation2D::setY(double y) {
    Translation2D::y = y;
}
