//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_TRANSLATION2D_H
#define TEENSYROSCONTROLLER_TRANSLATION2D_H

#include "Rotation2D.h"

class Translation2D {
public:
    Translation2D(double x, double y);
    constexpr Translation2D() = default;

    double distance(const Translation2D &rhs) const;

    Translation2D rotateBy(const Rotation2D& other) const;

    double norm() const;

    double X() const;

    void setX(double x);

    double Y() const;

    void setY(double y);

    bool operator==(const Translation2D &rhs) const;

    bool operator!=(const Translation2D &rhs) const;

    Translation2D operator+(const Translation2D &other) const;

    Translation2D& operator+=(const Translation2D &other);

    Translation2D operator-(const Translation2D &other) const;

    Translation2D operator-() const;

    Translation2D& operator-=(const Translation2D &other);

    Translation2D operator*(double scalar) const;

    Translation2D& operator*=(double scalar);

    Translation2D operator/(double scalar) const;

    Translation2D& operator/=(double scalar);

private:
    double x{0};
    double y{0};
};


#endif //TEENSYROSCONTROLLER_TRANSLATION2D_H
