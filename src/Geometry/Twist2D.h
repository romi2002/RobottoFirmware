//
// Created by abiel on 7/28/20.
//

#ifndef TEENSYROSCONTROLLER_TWIST2D_H
#define TEENSYROSCONTROLLER_TWIST2D_H


class Twist2D {
public:
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
