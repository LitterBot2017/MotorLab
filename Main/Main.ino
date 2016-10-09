#include <ros.h>
#include <std_msgs/Int16.h>
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "LightGate.h"
#include "Servo.h"
#include "Thermistor.h"

Servo testServo;

ros::NodeHandle sensorNode;



std_msgs::Int16 ultraMsg;
std_msgs::Int16 irMsg;
std_msgs::Int16 lightGateMsg;
std_msgs::Int16 tempMsg;
ros::Publisher ultraDist("ultra_dist",&ultraMsg);
ros::Publisher irDist("ir_dist",&irMsg);
ros::Publisher lightGate("light_gate",&lightGateMsg);
ros::Publisher tempRead("temp_read",&tempMsg);

Ultrasonic ultraSensor(A0);
IRSensor irSensor(A1);
Thermistor tempSensor(A2);
LightGate lightGateSensor(7);

void ultraCb(const std_msgs::Int16& ultra_msg)
{
  int val=ultra_msg.data%180;
  testServo.write(val); 
}

ros::Subscriber <std_msgs::Int16> sub("ir_dist",&ultraCb);


void setup() {
  // put your setup code here, to run once:
  sensorNode.initNode();
  sensorNode.advertise(ultraDist);
  sensorNode.advertise(irDist);
  sensorNode.advertise(lightGate);
  sensorNode.advertise(tempRead);

  testServo.attach(9);

  sensorNode.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  ultraMsg.data=ultraSensor.filteredReading();
  irMsg.data=irSensor.distanceReading();
  lightGateMsg.data=lightGateSensor.getState();
  tempMsg.data=tempSensor.gettemperature();
  
  ultraDist.publish(&ultraMsg);
  irDist.publish(&irMsg);
  lightGate.publish(&lightGateMsg);
  tempRead.publish(&tempMsg);
  
  sensorNode.spinOnce();
  delay(100);
}
