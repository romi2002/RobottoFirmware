#include "BatteryPublisherTask.h"

BatteryPublisherTask::BatteryPublisherTask(ros::NodeHandle *nh, TickType_t waitTime) : Thread("BatteryPubTask", 256,
                                                                                              BATTERY_TASK_PRIORITY),
                                                                                       currentSensorVoltagePub(
                                                                                               "currentSensorVoltage",
                                                                                               &currentSensorVoltageMsg),
                                                                                       energyUsedPub("energyUsed",
                                                                                                     &energyUsedMsg),
                                                                                       batteryStatePub(
                                                                                               "batteryState",
                                                                                               &batteryStateMsg) {
    this->nh = nh;
    this->waitTime = waitTime;

    profilerIt = profiler.initProfiler("BatteryPublisherTask");

    Start();
}

[[noreturn]] void BatteryPublisherTask::Run() {
    nh->advertise(currentSensorVoltagePub);
    nh->advertise(energyUsedPub);
    nh->advertise(batteryStatePub);
    long startTime=0;

    while (true) {
        double currentSensorVoltage = ((double) analogRead(PinAssignments::CURRENT_SENSOR_PIN) / 4096) * 3.3;
        double current = (currentSensorVoltage - 1.65) / 0.055;

        double batteryVoltage = (((double) analogRead(PinAssignments::BATTERY_VOLTAGE_PIN) / 4096) * 3.3);
        batteryVoltage *= ((1000.0 + 300.0) / 300.0);
        outData.batteryVoltage = batteryVoltage;
        //batteryVoltage *= 0.95;


        double energy = ((current * (dt / 1000.0)) / 60.0) / 60.0; //mAh
        energyUsed += energy;

        vTaskDelay(waitTime);
        dt = 0;

        TaskProfiler::updateProfiler(profilerIt);
    }
}