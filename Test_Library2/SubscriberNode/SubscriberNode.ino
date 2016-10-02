#include <ros.h>
#include <std_msgs/Int16.h>
#include "Servo.h"

Servo testServo;

ros::NodeHandle motorNode;

void ultraCb(const std_msgs::Int16& ultra_msg)
{
  int val=(ultra_msg.data,0,400,0,180);
  testServo.write(val); 
}

ros::Subscriber <std_msgs::Int16> sub("ultra_dist",&ultraCb);

void setup() {
  // put your setup code here, to run once:
  testServo.attach(9);

  motorNode.initNode();
  motorNode.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  motorNode.spinOnce();
  delay(100);
}
