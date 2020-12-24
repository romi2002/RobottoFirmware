//
// Created by abiel on 11/14/20.
//

#include "IMUTask.h"

#include <cmath>
#include "Alarms.h"
#include "Globals.h"
#include "I2Cdev.h"
#include "IMU.h"
#include "Sensor_cal.h"
#include "Types.h"
#include "USFSMAX.h"
#include "imu_config.h"
#include "def.h"

#include "geometry_msgs/Quaternion.h"

geometry_msgs::Quaternion imuQuat;
double imuYaw;

#include <TeensyDebug.h>

bool IMUTask::printRPY = false;
bool IMUTask::printDHIStatus = false;
bool IMUTask::doResetDHI = false;

IMUTask::IMUTask(ReadWriteLockPreferWriter *i2cLock, TickType_t tickDelay)
        : Thread("IMUTask", configMINIMAL_STACK_SIZE, IMU_TASK_PRIORITY) {
    this->tickDelay = tickDelay;

    this->i2cLock = i2cLock;
    imuQuat = geometry_msgs::Quaternion();
    imuYaw = 0;

    i2c_0 = new I2Cdev(&Wire);
    UFSMAX_0 = new USFSMAX(i2c_0, 0);
    imu_0 = new IMU(UFSMAX_0, 0);
    sensor_cal = new Sensor_cal(i2c_0, UFSMAX_0, 0);

    digitalWrite(PinAssignments::IMU_RST_PIN, HIGH);
    delay(5000);

    ssize_t ret = UFSMAX_0->init_USFSMAX();
    delay(100);
    while(ret != 0){
        //Reset until ret is good
        delay(100);
        digitalWrite(PinAssignments::DEBUG_PIN, LOW);
        digitalWrite(PinAssignments::IMU_RST_PIN, LOW);
        delay(100);
        digitalWrite(PinAssignments::IMU_RST_PIN, HIGH);
        digitalWrite(PinAssignments::DEBUG_PIN, HIGH);
        delay(1000);
        ret = UFSMAX_0->init_USFSMAX();
        delay(200);
    }
    //delay(1000);
    Wire.setClock(1 * 100);
    digitalWrite(PinAssignments::IMU_RST_PIN, HIGH);
    delay(100);

    attachInterrupt(PinAssignments::IMU_INT_PIN, DRDY_handler_0, RISING);

    Mv_Cal = M_V;  // Vertical geomagnetic field component
    Mh_Cal = M_H;  // Horizontal geomagnetic field component
    M_Cal =
            std::sqrt(Mv_Cal * Mv_Cal + Mh_Cal * Mh_Cal);  // Geomagnetic field strength
    Del_Cal = std::atan(Mv_Cal / Mh_Cal);

    profilerIt = profiler.initProfiler("IMUTask");

    Start();
}

void IMUTask::ProcEventStatus(I2Cdev *i2c_BUS, uint8_t sensorNUM) {
    uint8_t temp[1];

    // Read algorithm status and event status
    i2c_BUS->readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, temp);
    eventStatus[sensorNUM] = temp[0];

    // Decode the event status to determine what data is ready and set the
    // appropriate DRDY fags
    if (eventStatus[sensorNUM] & 0x01) Gyro_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x02) Acc_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x04) Mag_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x08) Baro_flag[sensorNUM] = 1;
    if (eventStatus[sensorNUM] & 0x10) Quat_flag[sensorNUM] = 1;
}

void IMUTask::FetchUSFSMAX_Data(USFSMAX *usfsmax, IMU *IMu, uint8_t sensorNUM) {

        uint8_t call_sensors = eventStatus[sensorNUM] & 0x0F;

        Acq_time = 0;
        Begin = micros();

    // Optimize the I2C read function with respect to whatever sensor data is
        // ready
        switch (call_sensors) {
            case 0x01:
                usfsmax->GyroAccel_getADC();
                break;
            case 0x02:
                usfsmax->GyroAccel_getADC();
                break;
            case 0x03:
                usfsmax->GyroAccel_getADC();
                break;
            case 0x07:
                usfsmax->GyroAccelMagBaro_getADC();
                break;
            case 0x0B:
                usfsmax->GyroAccelMagBaro_getADC();
                break;
            case 0x0F:
                usfsmax->GyroAccelMagBaro_getADC();
                break;
            case 0x0C:
                usfsmax->MagBaro_getADC();
                break;
            case 0x04:
                usfsmax->MAG_getADC();
                break;
            case 0x08:
                usfsmax->BARO_getADC();
                break;
            default:
                break;
        };
        Acq_time += micros() - Begin;

        if (Mag_flag[sensorNUM]) {

            if (ScaledSensorDataFlag)  // Calibration data is applied in the
                // coprocessor; just scale
            {
                for (uint8_t i = 0; i < 3; i++) {
                    magData[sensorNUM][i] =
                            ((float) magADC[sensorNUM][i]) * UT_per_Count;
                }
            } else  // Calibration data applied locally
            {
                sensor_cal->apply_adv_calibration(
                        ellipsoid_magcal[sensorNUM], magADC[sensorNUM], UT_per_Count,
                        mag_calData[sensorNUM]);
                sensor_cal->apply_adv_calibration(final_magcal[sensorNUM],
                                                           mag_calData[sensorNUM],
                                                           1.0f, sensor_point);
                MAG_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
            }
            Mag_flag[sensorNUM] = 0;
        }
        if (Acc_flag[sensorNUM]) {

            if (ScaledSensorDataFlag)  // Calibration data is applied in the
                // coprocessor; just scale
            {
                for (uint8_t i = 0; i < 3; i++) {
                    accData[sensorNUM][i] =
                            ((float) accADC[sensorNUM][i]) * g_per_count;
                }
            } else  // Calibration data applied locally
            {
                sensor_cal->apply_adv_calibration(
                        accelcal[sensorNUM], accADC[sensorNUM], g_per_count,
                        sensor_point);
                ACC_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
            }
            Acc_flag[sensorNUM] = 0;
        }
        if (Gyro_flag[sensorNUM] == 1) {

            if (ScaledSensorDataFlag)  // Calibration data is applied in the
                // coprocessor; just scale
            {
                for (uint8_t i = 0; i < 3; i++) {
                    gyroData[sensorNUM][i] =
                            ((float) gyroADC[sensorNUM][i]) * dps_per_count;
                }
            } else  // Calibration data applied locally
            {
                sensor_cal->apply_adv_calibration(
                        gyrocal[sensorNUM], gyroADC[sensorNUM], dps_per_count,
                        sensor_point);
                GYRO_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
            }

            // Call alternative (Madgwick or Mahony) IMU fusion filter
            IMu->compute_Alternate_IMU();
            Gyro_flag[sensorNUM] = 0;
        }

        if (Quat_flag[sensorNUM] == 1) {
            IMu->computeIMU();
            Quat_flag[sensorNUM] = 0;
        }

}

[[noreturn]] void IMUTask::Run() {

    while (true) {
        if (dataReady) {
            dataReady = 0;

            if(printDHIStatus){
                cal_status[0] = i2c_0->readByte(MAX32660_SLV_ADDR, CALIBRATION_STATUS);
                UFSMAX_0->getDHI_Rsq();
                SerialUSB2.print("Dynamic Hard Iron Correction Valid = ");
                SerialUSB2.println(cal_status[0] & 0x80);                                                                        // DHI correction status
                SerialUSB2.print("Dynamic Hard Iron Fit R-square = ");
                SerialUSB2.println(Rsq, 4);
                //printDHIStatus = false;
            }

            if(doResetDHI){
                UFSMAX_0->Reset_DHI();
                doResetDHI = false;
            }

            noInterrupts();
            i2cLock->WriterLock();
            ProcEventStatus(i2c_0, 0);
            FetchUSFSMAX_Data(UFSMAX_0, imu_0, 0);
            i2cLock->WriterUnlock();
            interrupts();

            imuQuat.x = qt[0][1];
            imuQuat.y = qt[0][2];
            imuQuat.z = qt[0][3];
            imuQuat.w = qt[0][0];

            imuYaw = heading[0];

            if(printRPY){
                if(RPYPrintCount++ > 5) {
                    //Print at a lower rate
                    SerialUSB2.print("Roll: ");
                    SerialUSB2.print(angle[0][0]);
                    SerialUSB2.print(", Pitch: ");
                    SerialUSB2.print(angle[0][1]);
                    SerialUSB2.print(", Yaw: ");
                    SerialUSB2.println(heading[0]);
                    RPYPrintCount = 0;
                }
            }

            //Serial.println(imuYaw);
        }

        TaskProfiler::updateProfiler(profilerIt);
        startTime = millis();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}