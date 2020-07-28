//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_ROTATION2D_H
#define TEENSYROSCONTROLLER_ROTATION2D_H

#include <cmath>

class Rotation2D {
public:
    constexpr Rotation2D() = default;

    explicit Rotation2D(double rad);

    Rotation2D(double x, double y);

    double rad() const {return value; };

    double deg() const {return (value / M_PI) * 180.0; };

    double cos() const {return m_cos; };

    double sin() const {return m_sin; };

    double tan() const {return m_sin / m_cos; };

    Rotation2D rotateBy(const Rotation2D &other) const;

    bool operator==(const Rotation2D &rhs) const;

    bool operator!=(const Rotation2D &rhs) const;

    Rotation2D operator+(const Rotation2D &other) const;

    Rotation2D& operator +=(const Rotation2D& other);

    Rotation2D operator-(const Rotation2D &other) const;

    Rotation2D operator-() const;

    Rotation2D& operator -=(const Rotation2D& other);

    Rotation2D operator*(double scalar) const;

private:
    double value = 0;
    double m_cos = 1;
    double m_sin = 0;
};


#endif //TEENSYROSCONTROLLER_ROTATION2D_H
