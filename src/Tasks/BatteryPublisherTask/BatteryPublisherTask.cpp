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

        currentSensorVoltageMsg.data = analogRead(PinAssignments::CURRENT_SENSOR_PIN);
        currentSensorVoltagePub.publish(&currentSensorVoltageMsg);

        double batteryVoltage = (((double) analogRead(PinAssignments::BATTERY_VOLTAGE_PIN) / 4096) * 3.3);
        batteryVoltage *= ((1000.0 + 300.0) / 300.0);
        //batteryVoltage *= 0.95;

        batteryStateMsg.voltage = batteryVoltage;
        batteryStateMsg.current = currentSensorVoltage;
        batteryStateMsg.present = true;
        batteryStateMsg.power_supply_status = 2; //Discharging
        batteryStateMsg.power_supply_health = 0;
        batteryStateMsg.power_supply_technology = 3;

        //batteryStatePub.publish(&batteryStateMsg);

        double energy = ((current * (dt / 1000.0)) / 60.0) / 60.0; //mAh
        energyUsed += energy;

        energyUsedMsg.data = energyUsed;
        //energyUsedPub.publish(&energyUsedMsg);

        vTaskDelay(waitTime);
        dt = 0;

        SerialUSB.print("Battery took: "); SerialUSB.println(millis()-startTime);
        startTime = millis();
    }
}