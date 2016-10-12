#include <ros.h>
#include <std_msgs/Int16.h>
#include <motorlab_msgs/MotorLab_Arduino.h>
#include <motorlab_msgs/MotorLab_PC.h>
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "LightGate.h"
#include "Servo.h"
#include "Thermistor.h"

const int button0_pin = 2;
int but0_old_state = LOW;
int but0_new_state = LOW;
int current_state = 0;

motorlab_msgs::MotorLab_Arduino ArduinoMsg;
ros::Publisher output_msg("ArduinoMsg",&ArduinoMsg);

//State Defines
#define NO_SENSE 0
#define THERM_SENSE 1
#define LGATE_SENSE 2
#define IRRNG_SENSE 4
#define ULTRA_SENSE 8

#define NO_ACT 0
#define M_POS_ACT 1
#define M_VEL_ACT 2
#define SERVO_ACT 4


// Actuators
Servo myServo;
// TODO: Initialize Stepper
// TODO: Initialize DC Motor

// Sensors
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
      ArduinoMsg.button_state=current_state*10;
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

//Gobal var for Servo Angle
int servo_angle = 0;

//State Variables
int sensor_state = 0;
int actuator_state = 0;

//Vars for mapping
int sensor_max = 0;
int sensor_min = 0;
int sensor_reading = 0;
int actuator_max = 0;
int actuator_min = 0;
int actuator_effort = 0;

// ROS things
ros::NodeHandle arduinoNode;
motorlab_msgs::MotorLab_Arduino ArduinoMsg;
motorlab_msgs::MotorLab_PC PCMsg;

void PC_callback(const motorlab_msgs::MotorLab_PC& pc_msg)
{
  PCMsg.stepper_angle = pc_msg.stepper_angle;
  PCMsg.motor_position_checked = pc_msg.motor_position_checked;
  PCMsg.motor_speed_checked = pc_msg.motor_speed_checked;
  PCMsg.servo_checked = pc_msg.servo_checked;
  PCMsg.thermistor_checked = pc_msg.thermistor_checked;
  PCMsg.light_gate_checked = pc_msg.light_gate_checked;
  PCMsg.ir_checked = pc_msg.ir_checked;
  PCMsg.ultra_checked = pc_msg.ultra_checked;
}

ros::Publisher output_msg("ArduinoMsg",&ArduinoMsg);
ros::Subscriber <motorlab_msgs::MotorLab_PC> input_msg("PCMsg",&PC_callback);

/************** Arduino Specific *********************/

void setup() {
  // ROS Initialization
  arduinoNode.initNode();
  arduinoNode.advertise(output_msg);
  
  arduinoNode.subscribe(input_msg);

  // Initialize Actuators
  myServo.attach(9);
  pinMode(button0_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button0_pin),button0_isr_change,CHANGE);
}

void loop() {
  // package the output message
  ArduinoMsg.dc_motor_position = 0;
  ArduinoMsg.dc_motor_speed = 0;
  ArduinoMsg.servo_position = servo_angle;
  ArduinoMsg.stepper_motor_position = 0;
  ArduinoMsg.temperature = tempSensor.gettemperature();
  ArduinoMsg.light_gate_state=lightGateSensor.getState();
  ArduinoMsg.ultrasonic_distance = ultraSensor.filteredReading();
  ArduinoMsg.ir_distance = irSensor.distanceReading();
  output_msg.publish(&ArduinoMsg);
  
  //Sensor State
  sensor_state = THERM_SENSE*PCMsg.thermistor_checked + 
                LGATE_SENSE*PCMsg.light_gate_checked + 
                IRRNG_SENSE*PCMsg.ir_checked + 
                ULTRA_SENSE*PCMsg.ultra_checked;
  switch(sensor_state){
    case THERM_SENSE:
      sensor_min = 400;
      sensor_max = 600;
      sensor_reading = ArduinoMsg.temperature;
      sensor_reading = constrain(sensor_reading, sensor_min, sensor_max);
      break;
    case LGATE_SENSE:
      break;
    case IRRNG_SENSE:
      break;
    case ULTRA_SENSE:
      break;
    default:
      sensor_min = -1;
      sensor_max = -1;
      break;
  }

  //Actuator State
  if ((sensor_max != -1) && (sensor_min != -1))//run only if sensor selection is valid
  {
    actuator_state = M_POS_ACT*PCMsg.motor_position_checked + M_VEL_ACT*PCMsg.motor_speed_checked + SERVO_ACT*PCMsg.servo_checked;
    switch(actuator_state){
      case M_POS_ACT:
        // actuator_effort = map(sensor_reading, sensor_min, sensor_max, actuator_min, actuator_max);
        break;
      case M_VEL_ACT:
        // actuator_effort = map(sensor_reading, sensor_min, sensor_max, actuator_min, actuator_max);
        break;
      case SERVO_ACT:
        actuator_min = 0;
        actuator_max = 180;
        actuator_effort = map(sensor_reading, sensor_min, sensor_max, actuator_min, actuator_max);
        servo_angle = actuator_effort;
        myServo.write(actuator_effort);
        break;
      default:
        myServo.write(0);
        break;
    }
  }



  arduinoNode.spinOnce();
  delay(100);
}
