//
// Created by abiel on 7/2/20.
//

#ifndef TEENSYROSCONTROLLER_LOG_H
#define TEENSYROSCONTROLLER_LOG_H

#include <string>
#include <cstdio>

//#define LOG_DEBUG

template<typename T>
void serialLog(const T &out) {
#ifdef LOG_DEBUG
    SerialUSB.print(out);
#endif
}

template<typename T>
void serialLogLn(const T &out) {
#ifdef LOG_DEBUG
    SerialUSB.println(out);
#endif
}

#endif //TEENSYROSCONTROLLER_LOG_H
