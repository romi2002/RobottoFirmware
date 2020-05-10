#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#define USE_USBCON

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

const uint8_t HEARTBEAT_PIN = 1;

ros::NodeHandle nh;

std_msgs::Float32 voltage_msg;
std_msgs::Float32 current_msg;
std_msgs::String runtimeStats_msg;

ros::Publisher voltage("voltage", &voltage_msg);
ros::Publisher current("current", &current_msg);
ros::Publisher runtimeStats("runtimeStats", &runtimeStats_msg);

static void heartbeatThread(void *arg){
  //pinMode(2, OUTPUT);

  while(1){
    vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
    digitalWrite(HEARTBEAT_PIN, HIGH);
    vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
    digitalWrite(HEARTBEAT_PIN, LOW);
  }
}

static void testThread(void *arg){
  while (1){
    double inputVoltage = ((double)analogRead(A8) / 4096) * 3.3;
    double currentIn = (inputVoltage - 1.65) / 0.055;

    voltage_msg.data = inputVoltage;
    voltage.publish(&voltage_msg);

    current_msg.data = currentIn;
    current.publish(&current_msg);

    vTaskDelay((20L * configTICK_RATE_HZ) / 1000L);
  }
}

static void rosSpin(void *arg){
  while (1){
    nh.spinOnce();
    vTaskDelay((20L * configTICK_RATE_HZ) / 1000L);
  }
}

static void runTimePublisher(void *arg){
  char buffer[1024];
  char *bufferTest = buffer;

  while (1){
    vTaskGetRunTimeStats(buffer);
    runtimeStats_msg.data = buffer;
    runtimeStats.publish(&runtimeStats_msg);

    vTaskDelay((20L * configTICK_RATE_HZ) / 1000L);
  }
}

void setup(){
  pinMode(HEARTBEAT_PIN, OUTPUT);

  analogReadResolution(12);

  nh.initNode();

  /**
   * ROS Advertise
   */
  nh.advertise(voltage);
  nh.advertise(current);
  nh.advertise(runtimeStats);

  /**
   * RTOS SETUP
   */

  //portBASE_TYPE s1 = xTaskCreate(testThread, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  // create task at priority one
  portBASE_TYPE s2 = xTaskCreate(rosSpin, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  portBASE_TYPE heartBeatTask = xTaskCreate(heartbeatThread, "HeartbeatThread", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  portBASE_TYPE runTimeTask = xTaskCreate(runTimePublisher, "RuntimeStatsPub", 2500, NULL, 3, NULL);

  //delay(1000);

  // if (heartBeatTask != pdPASS){
  //   while (1){
  //     digitalWrite(HEARTBEAT_PIN, LOW);
  //   }
  // }

  vTaskStartScheduler();
}

void loop(){
  ;
}