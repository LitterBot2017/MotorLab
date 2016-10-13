#include "motor_velocity_controller.h"
#include "Arduino.h"
#include <Encoder.h>

MotorVelocityController::MotorVelocityController(byte pMotorEncoderAPort, byte pMotorEncoderBPort, byte pMotorL1Port, byte pMotorL2Port, byte pMotorSpeedPort, MotorController* pMotorController) {

  mMotorEncoderAPort = pMotorEncoderAPort;
  mMotorEncoderBPort = pMotorEncoderBPort;
  mMotorL1Port = pMotorL1Port;
  mMotorL2Port = pMotorL2Port;
  mMotorSpeedPort = pMotorSpeedPort;

  // Motor pin setup
  pinMode(mMotorL1Port, OUTPUT);
  pinMode(mMotorL2Port, OUTPUT);
  pinMode(mMotorSpeedPort, OUTPUT);

  // Motor Encoder Pin Setup
  pinMode(mMotorEncoderAPort, INPUT);
  pinMode(mMotorEncoderBPort, INPUT);

  myEnc = new Encoder(mMotorEncoderAPort, mMotorEncoderBPort);
  motorController = pMotorController;

  timeLast = millis();
  errorLast = 0;
  lastAngle = getAngle();
}

void MotorVelocityController::setVelocity(int desiredRPM) {
/*
  desiredVelocity = desiredRPM;
  currentVelocity = getVeloctiyRPM();

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
  }

  timeLast = timeNow;
  errorLast = errorNow;

  if (abs(errorNow) <= 1) {
    digitalWrite(mMotorL1Port, LOW);
    digitalWrite(mMotorL2Port, HIGH);
    analogWrite(mMotorSpeedPort, 0);
  } else {
    (*motorController).turnMotor(control);
  }*/
}

int MotorVelocityController::getVeloctiyRPM() {

  currentAngle = getAngle();

  if (abs(lastAngle - currentAngle) > 0) {

    timeNow = millis();
    timeDiff = timeNow - timeLast;
    timeLast = timeNow;

    double dps = ((currentAngle - lastAngle) / (timeDiff / 1000.0));
    currentVelocity = ((int) degreesPerSecondToRPM(dps));
    lastAngle = currentAngle;
  }

  return currentVelocity;

  /**********************************************************
  // One potential implementation; uses delays = inefficient

  int initAngle = Motor::getAngle();
  int initTime = millis();

  delay(100);

  int finalAngle = Motor::getAngle();
  int finalTime = millis();

  int angleDiff = finalAngle - initAngle;
  int timeDiff = finalTime - initTime;

  double dpsVelocity = (angleDiff / (timeDiff / 1000.0));
  return Motor::degreesPerSecondToRPM(dpsVelocity)
  *************************************************************/
}

double MotorVelocityController::degreesPerSecondToRPM(double dps) {
  return ((dps * 60) / 360);
}

int MotorVelocityController::getAngle() {

  currentAngle = (((*myEnc).read()/2) % 360);
  return currentAngle;
}

