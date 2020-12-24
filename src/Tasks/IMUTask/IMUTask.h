//
// Created by abiel on 5/10/20.
//

#ifndef IMUTASK_H
#define IMUTASK_H

#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include "PinAssignments.h"
#include "config.h"

#include "read_write_lock.hpp"
#include "thread.hpp"

#include "Utils/TaskProfiler/TaskProfiler.h"

using namespace cpp_freertos;

class I2Cdev;
class USFSMAX;
class IMU;
class Sensor_cal;

static volatile int dataReady = 0;

class IMUTask : public Thread {
public:
    explicit IMUTask(ReadWriteLockPreferWriter *i2cLock,
                     TickType_t tickDelay = DEFAULT_WAIT_TIME);

    static void setPrintRPY(bool status){
        printRPY = status;
    }

    static void setPrintDHIStatus(){
        printDHIStatus = !printDHIStatus;
    }

    static void resetDHICorrector(){
        doResetDHI = true;
    }

protected:
    TickType_t tickDelay;
    ReadWriteLockPreferWriter *i2cLock;

    [[noreturn]] void Run() override;

private:
    I2Cdev *i2c_0;
    USFSMAX *UFSMAX_0;
    IMU *imu_0;
    Sensor_cal *sensor_cal;
    long startTime{0};

    static bool printRPY;
    static bool printDHIStatus;
    static bool doResetDHI;
    int RPYPrintCount{0};

private:
    void ProcEventStatus(I2Cdev *i2c_BUS, uint8_t sensorNUM);

    void FetchUSFSMAX_Data(USFSMAX *usfsmax, IMU *IMu, uint8_t sensorNUM);

    TaskProfiler& profiler = TaskProfiler::getInstance();
    TaskProfilerIt profilerIt;
};

inline void DRDY_handler_0() { dataReady = 1; }

#endif