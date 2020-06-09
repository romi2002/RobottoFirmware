#include "BatteryPublisherTask.h"

BatteryPublisherTask::BatteryPublisherTask(ros::NodeHandle *nh, TickType_t waitTime) : Thread("BatteryPubTask", 256,
                                                                                              BATTERY_TASK_PRIORITY),
                                                                                       currentSensorVoltagePub(
                                                                                               "currentSensorVoltage",
                                                                                               &currentSensorVoltageMsg),
                                                                                       currentSensorPub("currentSensor",
                                                                                                        &currentSensorMsg),
                                                                                       batteryVoltagePub(
                                                                                               "currentSensorVoltage",
                                                                                               &currentSensorVoltageMsg) {
    this->nh = nh;
    this->waitTime = waitTime;
    Start();
}

[[noreturn]] void BatteryPublisherTask::Run() {
    nh->advertise(currentSensorVoltagePub);
    nh->advertise(currentSensorPub);
    nh->advertise(batteryVoltagePub);

    while (true) {
        double inputVoltage = ((double) analogRead(CURRENT_SENSOR_PIN) / 4096) * 3.3;
        double currentIn = (inputVoltage - 1.65) / 0.055;

        currentSensorVoltageMsg.data = inputVoltage;
        currentSensorVoltagePub.publish(&currentSensorVoltageMsg);

        currentSensorMsg.data = currentIn;
        currentSensorPub.publish(&currentSensorMsg);

        batteryVoltageMsg.data = (((double) analogRead(BATTERY_VOLTAGE_PIN) / 4096) * 3.3);
        batteryVoltageMsg.data *= ((270.0 + 70.0) / 70.0);
        batteryVoltageMsg.data *= 0.95; //(Calib)

        batteryVoltagePub.publish(&batteryVoltageMsg);

        vTaskDelay(waitTime);
    }
}