//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_TWIST2D_H
#define TEENSYROSCONTROLLER_TWIST2D_H


class Twist2D {
public:
    constexpr Twist2D() = default;

    Twist2D(double dx, double dy, double dtheta){
        this->dx = dx;
        this->dy = dy;
        this->dtheta = dtheta;
    }

    Twist2D(const Twist2D &rhs){
        Twist2D(rhs.dx, rhs.dy, rhs.dtheta);
    }

    double dx{0};
    double dy{0};
    double dtheta{0};

    bool operator==(const Twist2D &rhs) const {
        return dx == rhs.dx &&
               dy == rhs.dy &&
               dtheta == rhs.dtheta;
    }

    bool operator!=(const Twist2D &rhs) const {
        return !(rhs == *this);
    }
};


#endif //TEENSYROSCONTROLLER_TWIST2D_H
