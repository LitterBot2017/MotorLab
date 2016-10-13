#include "motor_position_controller.h"
#include "Arduino.h"
#include <Encoder.h>

MotorPositionController::MotorPositionController(Encoder* pEncoder, byte pMotorL1Port, byte pMotorL2Port, byte pMotorSpeedPort, MotorController* pMotorController) {

  mMotorL1Port = pMotorL1Port;
  mMotorL2Port = pMotorL2Port;
  mMotorSpeedPort = pMotorSpeedPort;

  // Motor pin setup
  pinMode(mMotorL1Port, OUTPUT);
  pinMode(mMotorL2Port, OUTPUT);
  pinMode(mMotorSpeedPort, OUTPUT);

  // Motor Encoder Pin Setup
  // pinMode(mMotorEncoderAPort, INPUT);
  // pinMode(mMotorEncoderBPort, INPUT);

  timeLast = millis();
  errorLast = 0;

  errorSum = 0;
  
  myEnc = pEncoder;
  motorController = pMotorController;
}

void MotorPositionController::setAngle(int desired) {

  desiredAngle = desired;
  currentAngle = getAngle();

  errorNow = desiredAngle - currentAngle;
  timeNow = millis();
  timeDiff = timeNow - timeLast;
  errorDiff = errorNow - errorLast;

  pControl = kp * errorNow;
  errorSum += (errorNow * timeDiff);
  iControl = ki * errorSum;
  dControl = kd * (errorDiff/timeDiff);
  control = pControl + iControl + dControl;

  if ((millis() % 1000) == 0) {
    Serial.println("errorNow: " + String(errorNow) + "; errorDiff: " + String(errorDiff) + "; control = " + String(control));
    Serial.println("currentAngle: " + String(currentAngle) + "; desiredAngle: " + String(desiredAngle));
  }

  timeLast = timeNow;
  errorLast = errorNow;

//  if (abs(errorNow) <= 1) {
//    digitalWrite(mMotorL1Port, HIGH);
//    digitalWrite(mMotorL2Port, HIGH);
//    digitalWrite(mMotorSpeedPort, HIGH);
//  } else {
    (*motorController).turnMotor(control);
//  }
}

int MotorPositionController::getAngle() {

  currentAngle = (((*myEnc).read()/2) % 720);
  return currentAngle;
}

