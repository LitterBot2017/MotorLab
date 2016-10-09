#include <ros.h>
#include <std_msgs/Int16.h>
#include <motorlab_msgs/MotorLab_Arduino.h>
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "LightGate.h"
#include "Servo.h"

Servo testServo;

ros::NodeHandle sensorNode;
motorlab_msgs::MotorLab_Arduino ArduinoMsg;


// std_msgs::Int16 ultraMsg;
// std_msgs::Int16 irMsg;
// std_msgs::Int16 lightGateMsg;

//ros::Publisher ultraDist("ultra_dist",&ultraMsg);
//ros::Publisher irDist("ir_dist",&irMsg);
//ros::Publisher lightGate("light_gate",&lightGateMsg);
ros::Publisher output_msg("ArduinoMsg",&ArduinoMsg);

Ultrasonic ultraSensor(A0);
IRSensor irSensor(A1);
LightGate lightGateSensor(7);

void ultraCb(const std_msgs::Int16& ultra_msg)
{
  int val=ultra_msg.data%180;
  //lightGateMsg.data=val;
  //lightGate.publish(&lightGateMsg);
  testServo.write(val); 
}

// ros::Subscriber <std_msgs::Int16> sub("ir_dist",&ultraCb);


void setup() {
  // put your setup code here, to run once:
  sensorNode.initNode();
//  sensorNode.advertise(ultraDist);
//  sensorNode.advertise(irDist);
//  sensorNode.advertise(lightGate);
  sensorNode.advertise(output_msg);

  testServo.attach(9);

  // sensorNode.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  // ultraMsg.data=ultraSensor.filteredReading();
  // irMsg.data=irSensor.distanceReading();
  // lightGateMsg.data=lightGateSensor.getState();
//  ultraDist.publish(&ultraMsg);
//  irDist.publish(&irMsg);
//  lightGate.publish(&lightGateMsg);
  ArduinoMsg.dc_motor_position = 0;
  ArduinoMsg.dc_motor_speed = 0;
  ArduinoMsg.servo_position = 0;
  ArduinoMsg.stepper_motor_position = 0;
  ArduinoMsg.temperature = 0;
  ArduinoMsg.light_gate_state = lightGateSensor.getState();
  ArduinoMsg.ultrasonic_distance = ultraSensor.filteredReading();
  ArduinoMsg.ir_distance = irSensor.distanceReading();
  output_msg.publish(&ArduinoMsg);
  sensorNode.spinOnce();
  delay(100);
}
