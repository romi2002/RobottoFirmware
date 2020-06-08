//
// Created by abiel on 6/7/20.
//

#ifndef TEENSYROSCONTROLLER_MOTORCONTROLLER_H
#define TEENSYROSCONTROLLER_MOTORCONTROLLER_H

#include "config.h"

#include "thread.hpp"
#include "read_write_lock.hpp"
using namespace cpp_freertos;

#include "VNH5019.h"
#include "QuadEncoder_Teensy4.h"

#include "PID/PIDConfig.h"

enum class MotorControlMode {
    PERCENTAGE,
    VELOCITY,
    POSITION
};

struct EncoderPinDefinitions {
    uint8_t channel;
    uint8_t phaseA;
    uint8_t phaseB;
};

struct MotorControllerConfig {
    VNH5019_PinDefinitions vnh5019PinDefinitions;
    EncoderPinDefinitions encoderPinDefinitions;

    double countsPerRev = 979.62;

    double positionFilterAlpha = 0.05;
    double velocityFilterAlpha = 0.05;

    double motorDeadband = 0.1;

    PIDConfig velocityPIDConfig, positionPIDConfig;
};

class MotorController : public Thread {
public:
    MotorController(const std::string &name, const MotorControllerConfig &config, TickType_t updateTime = DEFAULT_WAIT_TIME);

    void set(double setpoint, MotorControlMode mode);

    double getVelocity() const;
    double getPosition() const;

protected:
    void Run() override;

private:
    MotorControllerConfig config;

    TickType_t updateTime;

    /**
     * Setpoint
     */
     double setpoint{0};
     MotorControlMode currentMode{MotorControlMode::PERCENTAGE};
     ReadWriteLock *setpointLock;

    /**
     * PID
     */
     PIDConfig velocityPIDConfig, positionPIDConfig;
    ReadWriteLock *pidConfigLock;

    /**
     * Encoder
     */
     QuadEncoder *encoder;
     double averageVelocity{0}, averagePosition{0};
     ReadWriteLock *averageVelocityLock, *averagePositionLock;

     /**
      * Motor driver
      */
      VNH5019 *motor;
};


#endif //TEENSYROSCONTROLLER_MOTORCONTROLLER_H
