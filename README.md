## RobottoFirmware
Firmware developed for the `Teensy 4.1` on a [RobottoPCB](https://github.com/romi2002/RobottoPCB). Handles communication with a single board computer (Jetson Nano or Raspberry Pi) using a modified version of `rosserial`. Implements motor control including PID loops and odometry.
### Features
* Runs on FreeRTOS
* Embedded serial console, can be used to get diagnostics and current system status from the Teensy.
* Mecanum wheel forward and inverse kinematics
* Odometry implementation
* Position and Velocity PID controllers
