//
// Created by abiel on 12/17/20.
//

#ifndef TEENSYROSCONTROLLER_ROLLINGAVERAGE_H
#define TEENSYROSCONTROLLER_ROLLINGAVERAGE_H

double exponentialMovingAverage(double avg, double new_val, double alpha = 1){
    return (alpha * new_val) + (1.0 - alpha) * avg;
}

#endif //TEENSYROSCONTROLLER_ROLLINGAVERAGE_H
