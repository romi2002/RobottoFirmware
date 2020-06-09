//
// Created by abiel on 6/7/20.
//

#ifndef TEENSYROSCONTROLLER_PIDCONFIG_H
#define TEENSYROSCONTROLLER_PIDCONFIG_H


struct PIDConfig {
    double p = 0;
    double i = 0;
    double d = 0;

    bool enableRampRate = false;
    double rampRate = 0;

    double deadband = 0;
};


#endif //TEENSYROSCONTROLLER_PIDCONFIG_H
