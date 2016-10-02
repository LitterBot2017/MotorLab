#include <ros.h>
#include <std_msgs/Int32.h>
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "LightGate.h"

Servo testServo;

ros::NodeHandle sensorNode;

ros::NodeHandle motorNode;

std_msgs::Int32 ultraMsg;
std_msgs::Int32 irMsg;
std_msgs::Int32 lightGateMsg;
ros::Publisher ultraDist("ultra_dist",&ultraMsg);
ros::Publisher irDist("ir_dist",&irMsg);
ros::Publisher lightGate("light_gate",&lightGateMsg);

Ultrasonic ultraSensor(A0);
IRSensor irSensor(A1);
LightGate lightGateSensor(7);

void ultraCb(const std_msgs::Int32& ultra_msg)
{
  int val=(ultra_msg.data,0,400,0,180);
  test_servo.write(val); 
}

ros::Subscriber <std_msgs::Int32> sub("ultra_dist",&ultraCb);

void setup() {
  // put your setup code here, to run once:
  sensorNode.initNode();
  sensorNode.advertise(ultraDist);
  sensorNode.advertise(irDist);
  sensorNode.advertise(lightGate);

  testServo.attach(9);

  motorNode.initNode();
  motorNode.subscribe(sub)
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
  motorNode.spinOnce();
  delay(100);
}
