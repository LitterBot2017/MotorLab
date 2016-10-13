#include "motor_controller.h"
#include "Arduino.h"
#include <Encoder.h>

MotorController::MotorController(byte pMotorL1Port, byte pMotorL2Port, byte pMotorSpeedPort) {

  mMotorL1Port = pMotorL1Port;
  mMotorL2Port = pMotorL2Port;
  mMotorSpeedPort = pMotorSpeedPort;

  // Motor pin setup
  pinMode(mMotorL1Port, OUTPUT);
  pinMode(mMotorL2Port, OUTPUT);
  pinMode(mMotorSpeedPort, OUTPUT);
}

void MotorController::turnMotor(int velocity) {

  if (velocity > 255) {
    velocity = 255;
  } else if (velocity < -255) {
    velocity = -255;
  }

//  if (velocity > 0 && velocity < 90) {
//    velocity = 90;
//  } else if (velocity < 0 && velocity > -90) {
//    velocity = -90;
//  }

//  if (abs(velocity) > 0) {
    if (velocity < 0) {
      digitalWrite(mMotorL1Port, HIGH);
      digitalWrite(mMotorL2Port, LOW);
    } else {
      digitalWrite(mMotorL1Port, LOW);
      digitalWrite(mMotorL2Port, HIGH);
    }
    analogWrite(mMotorSpeedPort, abs(velocity));
//  }
}

