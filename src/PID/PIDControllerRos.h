//
// Created by abiel on 6/8/20.
//

#ifndef TEENSYROSCONTROLLER_PIDCONTROLLERROS_H
#define TEENSYROSCONTROLLER_PIDCONTROLLERROS_H

#include "PIDController.h"
#include "control_msgs/PidState.h"

namespace PIDControllerROS{
    control_msgs::PidState getPIDState(PIDController const * controller);
    PIDConfig getPIDConfig(const control_msgs::PidState &state);
}

#endif //TEENSYROSCONTROLLER_PIDCONTROLLERROS_H
