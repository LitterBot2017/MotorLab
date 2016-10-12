#include <ros.h>
#include <std_msgs/Int16.h>
#include <motorlab_msgs/MotorLab_Arduino.h>
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "LightGate.h"
#include "Servo.h"
#include "Thermistor.h"

Servo testServo;
const int button0_pin = 2;
int but0_old_state = LOW;
int but0_new_state = LOW;
int current_state = 0;

ros::NodeHandle sensorNode;
motorlab_msgs::MotorLab_Arduino ArduinoMsg;
ros::Publisher output_msg("ArduinoMsg",&ArduinoMsg);

Ultrasonic ultraSensor(A0);
IRSensor irSensor(A1);
Thermistor tempSensor(A2);
LightGate lightGateSensor(7);

unsigned long time0 = millis();

void but0_state()
{
  if (but0_new_state == HIGH && but0_old_state == LOW)
  {
    if (millis() - time0 > 100)
    {
      time0 = millis();
      but0_old_state = but0_new_state;
    }
  }
  if (but0_new_state == LOW && but0_old_state == HIGH)
  {
    if (millis() - time0 > 100)
    {
      time0 = millis();
      but0_old_state = but0_new_state;
      current_state++;
      current_state %= 2;
      ArduinoMsg.light_gate_state=current_state*10;
    }
  }
}

void button0_isr_change()
{
  current_state++; 
  if(but0_new_state==HIGH)
  {
    but0_new_state=LOW;
  }
  else
  {
    but0_new_state=HIGH;
  }
  but0_state();
}

void ultraCb(const std_msgs::Int16& ultra_msg)
{
  int val=ultra_msg.data%180;
  testServo.write(val); 
}

void setup() {
  // put your setup code here, to run once:
  sensorNode.initNode();

  sensorNode.advertise(output_msg);

  testServo.attach(9);
  pinMode(button0_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button0_pin),button0_isr_change,CHANGE);

}

void loop() {
  ArduinoMsg.dc_motor_position = 0;
  ArduinoMsg.dc_motor_speed = 0;
  ArduinoMsg.servo_position = 0;
  ArduinoMsg.stepper_motor_position = 0;
  ArduinoMsg.temperature = tempSensor.gettemperature();
  //lightGateSensor.getState();
  ArduinoMsg.ultrasonic_distance = ultraSensor.filteredReading();
  ArduinoMsg.ir_distance = irSensor.distanceReading();
  output_msg.publish(&ArduinoMsg);
  sensorNode.spinOnce();
  delay(100);
}
