#include "BatteryPublisherTask.h"

[[noreturn]] void batteryPublisherTask(void *arg){
    ros::NodeHandle *nh = (ros::NodeHandle*) arg;

    nh->advertise(currentSensorVoltagePub);
    nh->advertise(currentSensorPub);
    nh->advertise(batteryVoltagePub);

    while (1){
        double inputVoltage = ((double)analogRead(CURRENT_SENSOR_PIN) / 4096) * 3.3;
        double currentIn = (inputVoltage - 1.65) / 0.055;

        currentSensorVoltageMsg.data = inputVoltage;
        currentSensorVoltagePub.publish(&currentSensorVoltageMsg);

        currentSensorMsg.data = currentIn;
        currentSensorPub.publish(&currentSensorMsg);

        batteryVoltageMsg.data = (((double) analogRead(BATTERY_VOLTAGE_PIN) / 4096) * 3.3);
        batteryVoltageMsg.data *= ((270.0 + 70.0) / 70.0);
        batteryVoltageMsg.data *= 0.95; //(Calib)

        batteryVoltagePub.publish(&batteryVoltageMsg);

        vTaskDelay(DEFAULT_WAIT_TIME);
    }
}