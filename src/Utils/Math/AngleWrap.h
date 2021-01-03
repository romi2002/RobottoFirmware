//
// Created by abiel on 1/1/21.
//

#ifndef TEENSYROSCONTROLLER_ANGLEWRAP_H
#define TEENSYROSCONTROLLER_ANGLEWRAP_H
#include <cmath>

static double wrapAngle(double angle) {
    angle = std::copysign(std::fmod(angle, 2 * M_PI), angle);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}
#endif //TEENSYROSCONTROLLER_ANGLEWRAP_H
