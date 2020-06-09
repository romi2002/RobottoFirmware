//
// Created by abiel on 6/8/20.
//

#include "PIDControllerRos.h"

control_msgs::PidState PIDControllerROS::getPIDState(const PIDController *controller) {
    control_msgs::PidState msg;
    msg.timestep = ros::Duration();
    msg.timestep.fromSec(controller->getLastTimestep());
    msg.error = controller->getError();
    msg.error_dot = controller->getErrorDot();

    msg.p_error = controller->getP_Error();
    msg.i_error = controller->getI_Error();
    msg.d_error = controller->getD_Error();

    msg.p_term = controller->getCurrentConfig().p;
    msg.i_term = controller->getCurrentConfig().i;
    msg.d_term = controller->getCurrentConfig().d;

    msg.output = controller->getLastOutput();

    return msg;
}

PIDConfig PIDControllerROS::getPIDConfig(const control_msgs::PidState &state){
    PIDConfig config;

    config.p = state.p_term;
    config.i = state.i_term;
    config.d = state.d_term;

    return config;
}