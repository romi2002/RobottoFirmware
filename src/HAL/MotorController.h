//
// Created by abiel on 6/7/20.
//

#ifndef TEENSYROSCONTROLLER_MOTORCONTROLLER_H
#define TEENSYROSCONTROLLER_MOTORCONTROLLER_H

#include "config.h"

#include "thread.hpp"
#include "read_write_lock.hpp"
#include "PID/PIDControllerRos.h"

using namespace cpp_freertos;

#include "VNH5019.h"
#include "QuadEncoder_Teensy4.h"

#include "PID/PIDConfig.h"
#include "PID/PIDController.h"
#include "EncoderPinAssignments.h"

enum class MotorControlMode {
    PERCENTAGE,
    VELOCITY,
    POSITION
};

class Adafruit_MCP23017;

struct MotorControllerConfig {
    VNH5019_PinAssignments vnh5019PinDefinitions;
    EncoderPinAssignments encoderPinDefinitions;

    Adafruit_MCP23017 *mcp;
    ReadWriteLockPreferWriter *mcpLock;

    double countsPerRev = 979.62;

    double positionFilterAlpha = 0.8;
    double velocityFilterAlpha = 0.035;

    PIDConfig velocityPIDConfig, positionPIDConfig;
};

class MotorController : public Thread {
public:
    MotorController(const std::string &name, const MotorControllerConfig &config,
                    TickType_t updateTime = DEFAULT_WAIT_TIME);

    void set(double setpoint, MotorControlMode mode);

    double getVelocity() const;

    double getPosition() const;

    control_msgs::PidState getVelPIDStatus() const;

    control_msgs::PidState getPosPIDStatus() const;

    PIDConfig getVelPIDConfig() const;

    void setVelPIDConfig(const PIDConfig &configIn);

    PIDConfig getPosPIDConfig() const;

    void setPosPIDConfig(const PIDConfig &configIn);

    ~MotorController() override;

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
    bool modeChanged = false;
    ReadWriteLock *setpointLock;

    /**
     * PID
     */
    PIDController *velocityController, *positionController;
    ReadWriteLock *velocityPIDLock, *positionPIDLock;

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
