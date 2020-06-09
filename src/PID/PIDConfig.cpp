//
// Created by abiel on 6/7/20.
//

#include "PIDConfig.h"

bool PIDConfig::operator==(const PIDConfig &rhs) const {
    return p == rhs.p and i == rhs.i and d == rhs.d;
}

bool PIDConfig::operator!=(const PIDConfig &rhs) const {
    return !(*this == rhs);
}