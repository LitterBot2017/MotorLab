#include <ros.h>
#include <std_msgs/Int16.h>
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "LightGate.h"
#include "Servo.h"

ros::NodeHandle sensorNode;


std_msgs::Int16 ultraMsg;
std_msgs::Int16 irMsg;
std_msgs::Int16 lightGateMsg;
ros::Publisher ultraDist("ultra_dist",&ultraMsg);
ros::Publisher irDist("ir_dist",&irMsg);
ros::Publisher lightGate("light_gate",&lightGateMsg);

Ultrasonic ultraSensor(A0);
IRSensor irSensor(A1);
LightGate lightGateSensor(7);

void setup() {
  // put your setup code here, to run once:
  sensorNode.initNode();
  sensorNode.advertise(ultraDist);
  sensorNode.advertise(irDist);
  sensorNode.advertise(lightGate);
}

void loop() {
  // put your main code here, to run repeatedly:
  ultraMsg.data=ultraSensor.filteredReading();
  irMsg.data=irSensor.distanceReading();
  lightGateMsg.data=lightGateSensor.getState();
  ultraDist.publish(&ultraMsg);
  irDist.publish(&irMsg);
  lightGate.publish(&lightGateMsg);
  sensorNode.spinOnce();
  delay(100);
}
