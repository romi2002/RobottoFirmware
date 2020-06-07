#pragma once

#include <FreeRTOS_TEENSY4.h>

const TickType_t DEFAULT_WAIT_TIME = (20L * configTICK_RATE_HZ) / 1000L;

/**
 * Task priorities
 */
const UBaseType_t HEARTBEAT_TASK_PRIORITY = tskIDLE_PRIORITY;
const UBaseType_t  BATTERY_TASK_PRIORITY = 1;
const UBaseType_t ROS_SPIN_TASK_PRIORITY = 2;