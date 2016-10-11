#include <ros.h>
#include <std_msgs/Int16.h>
#include <motorlab_msgs/MotorLab_Arduino.h>
#include "Ultrasonic.h"
#include "IRSensor.h"
#include "LightGate.h"
#include "Servo.h"
#include "Thermistor.h"

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

//Gobal var for Servo Angle
int servo_angle = 0;

//State Variables
int sensor_state = 0;
int actuator_state = 0;

//Vars for mapping
int sensor_max = 0;
int sensor_min = 0;
int actuator_max = 0;
int actuator_min = 0;

// ROS things
ros::NodeHandle arduinoNode;
motorlab_msgs::MotorLab_Arduino ArduinoMsg;
motorlab_msgs::MotorLab_PC PCMsg;

ros::Publisher output_msg("ArduinoMsg",&ArduinoMsg);
ros::Subscriber <motorlab_msgs::MotorLab_PC> input_msg("PCMsg",&PC_callback);

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

/************** Arduino Specific *********************/

void setup() {
  // ROS Initialization
  arduinoNode.initNode();
  arduinoNode.advertise(output_msg);
  
  arduinoNode.subscribe(sub);

  // Initialize Actuators
  myServo.attach(9);
}

void loop() {
  // package the output message
  ArduinoMsg.dc_motor_position = 0;
  ArduinoMsg.dc_motor_speed = 0;
  ArduinoMsg.servo_position = 0;
  ArduinoMsg.stepper_motor_position = 0;
  ArduinoMsg.temperature = tempSensor.gettemperature();
  ArduinoMsg.light_gate_state = lightGateSensor.getState();
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
        break;
      case M_VEL_ACT:
        break;
      case SERVO_ACT:

        break;
      default:
        break;
    }
  }



  arduinoNode.spinOnce();
  delay(100);
}
