//
// Created by abiel on 7/29/20.
//

#ifndef TEENSYROSCONTROLLER_POSE2D_H
#define TEENSYROSCONTROLLER_POSE2D_H


class Pose2D {
public:
    constexpr Pose2D() = default;

    Pose2D(double x, double y, double theta){
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    Pose2D operator+(const Pose2D &other) const;

    Pose2D operator +=(const Pose2D &other);

    double x{0}, y{0}, theta{0};
};


#endif //TEENSYROSCONTROLLER_POSE2D_H
