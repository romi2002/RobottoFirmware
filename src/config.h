#pragma once

#include "FreeRTOS_TEENSY4.h"
const TickType_t DEFAULT_WAIT_TIME = pdMS_TO_TICKS(10);

/**
 * Task priorities
 */
const UBaseType_t HEARTBEAT_TASK_PRIORITY = tskIDLE_PRIORITY;
const UBaseType_t BATTERY_TASK_PRIORITY = 1;
const UBaseType_t ROS_SPIN_TASK_PRIORITY = 2;
const UBaseType_t MOTOR_OUTPUT_TEST_TASK_PRIORITY = 3;
const UBaseType_t MOTOR_DRIVER_TASK_PRIORITY = 4;
const UBaseType_t MECANUM_TASK_PRIORITY = 5;