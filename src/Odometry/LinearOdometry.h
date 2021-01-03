//
// Created by abiel on 1/1/21.
//

#ifndef TEENSYROSCONTROLLER_LINEALODOMETRY_H
#define TEENSYROSCONTROLLER_LINEALODOMETRY_H

#include "Geometry/Twist2D.h"
#include "Geometry/Pose2D.h"

class LinearOdometry {
public:
    LinearOdometry();

    void update(const Twist2D &newPosition, double yaw);

    Pose2D getPose() const;

private:
    Twist2D lastPos{0,0,0};
    Pose2D currentPose{0,0,0};
};


#endif //TEENSYROSCONTROLLER_LINEALODOMETRY_H
