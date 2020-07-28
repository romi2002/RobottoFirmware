//
// Created by abiel on 7/28/20.
//

#include "Rotation2D.h"

bool Rotation2D::operator==(const Rotation2D &rhs) const {
    return value == rhs.value &&
           m_cos == rhs.m_cos &&
           m_sin == rhs.m_sin;
}

bool Rotation2D::operator!=(const Rotation2D &rhs) const {
    return !(rhs == *this);
}

Rotation2D::Rotation2D(double rad) {
    this->value = rad;
    this->m_cos = std::cos(rad);
    this->m_sin = std::sin(rad);
}

Rotation2D::Rotation2D(double x, double y) {
    const auto mag = std::hypot(x, y);
    if(mag > 1e-6){
        m_sin = y / mag;
        m_cos = x /mag;
    } else {
        m_sin = 0.0;
        m_cos = 1.0;
    }

    value = std::atan2(m_sin, m_cos);
}

Rotation2D Rotation2D::rotateBy(const Rotation2D &other) const {
    return {
        cos() * other.cos() - sin() * other.sin(),
        cos() * other.sin() + sin() * other.cos()
    };
}

Rotation2D Rotation2D::operator+(const Rotation2D &other) const {
    return rotateBy(other);
}

Rotation2D& Rotation2D::operator+=(const Rotation2D& other) {
    double cosTemp = cos() * other.cos() - sin() * other.sin();
    double sinTemp = sin() * other.sin() + sin() * other.cos();
    m_cos = cosTemp;
    m_sin = sinTemp;
    value = std::atan2(m_sin, m_cos);
    return *this;
}

Rotation2D Rotation2D::operator-(const Rotation2D &other) const {
    return *this + -other;
}

Rotation2D & Rotation2D::operator-=(const Rotation2D &other) {
    *this += -other;
    return *this;
}

Rotation2D Rotation2D::operator-() const {
    return Rotation2D(-value);
}

Rotation2D Rotation2D::operator*(double scalar) const {
    return Rotation2D(value * scalar);
}