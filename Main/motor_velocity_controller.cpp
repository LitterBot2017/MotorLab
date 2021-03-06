#include "motor_velocity_controller.h"
#include "Arduino.h"
#include <Encoder.h>

MotorVelocityController::MotorVelocityController(Encoder* pEncoder, byte pMotorL1Port, byte pMotorL2Port, byte pMotorSpeedPort, MotorController* pMotorController) {

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

  myEnc = pEncoder;
  motorController = pMotorController;

  pidTimeLast = millis();
  timeLast = millis();
  errorLast = 0;
  lastAngle = 0;
  currentAngle = 0;
  currentVelocity = 0;

  pidCurrAngle = 0;
  pidLastAngle = 0;
}

/*void MotorVelocityController::setVelocity(int desired) {

  desiredVelocity = desired;
  currentVelocity = getVeloctiyRPM();
  Serial.println("setVel -- currentVel = " + String(currentVelocity));

  pidCurrAngle = getAngle();
  if ((abs(pidCurrAngle - pidLastAngle) % desired) == 0) {
    Serial.println("if check = " + String(abs(pidCurrAngle - pidLastAngle) % desired));

    errorNow = desiredVelocity - currentVelocity;
    pidTimeNow = millis();
    pidTimeDiff = pidTimeNow - pidTimeLast;
    errorDiff = errorNow - errorLast;
  
    pControl = kp * errorNow;
    errorSum += (errorNow * pidTimeDiff);
    iControl = ki * errorSum;
    dControl = kd * (errorDiff/pidTimeDiff);
    control = pControl + iControl + dControl;
  
    pidTimeLast = pidTimeNow;
    errorLast = errorNow;

    pidLastAngle = pidCurrAngle;
    Serial.println("errorNow: " + String(errorNow) + "; errorDiff: " + String(errorDiff) + "; control = " + String(control) + "; currVel = " + String(currentVelocity) + "; desiredVelocity " + desiredVelocity);
  }

  (*motorController).turnMotor(control);
} */

void MotorVelocityController::setVelocity(int desired) {

  if (desired != desiredVelocity) {
    if (desired > 0 && desired <= 10) {
      control = 30;
    } else if (desired > 10 && desired <= 20) {
      control = 45;
    } else if (desired > 20 && desired <= 30) {
      control =  55;
    } else if (desired > 30 && desired <= 40) {
      control = 67;
    } else if (desired > 40 && desired <= 50) {
      control = 82;
    } else if (desired > 50 && desired <= 60) {
      control = 110;
    } else if (desired > 60 && desired <= 70) {
      control = 180;
    } else if (desired < 0 && desired >= -10) {
      control = -30;
    } else if (desired < -10 && desired >= -20) {
      control = -45;
    } else if (desired < -20 && desired >= -30) {
      control =  -57;
    } else if (desired < -30 && desired >= -40) {
      control = -67;
    } else if (desired < -40 && desired >= -50) {
      control = -82;
    } else if (desired < -50 && desired >= -60) {
      control = -110;
    } else if (desired < -60 && desired >= -70) {
      control = -145;
    }
  }

  desiredVelocity = desired;
  currentVelocity = getVeloctiyRPM();
  if (desiredVelocity == 0) {
    control = 0;
  } else if (currentVelocity < desiredVelocity) {
    control++;
  } else if (currentVelocity > desiredVelocity) {
    int diff = abs(currentVelocity - desiredVelocity) * 3;
    control--;
  }
  control = control % 256;
    
  Serial.println("Desired = " + String(desiredVelocity) + "; currentVelocity = " + String(currentVelocity) + "; control = " + String(control));

  (*motorController).turnMotor(control);
}

int MotorVelocityController::getVeloctiyRPM() {

  currentAngle = getAngle();
  //Serial.println("lastANgle = " + String(lastAngle) + "; currentANgle = " + String(currentAngle));
  if (abs(lastAngle - currentAngle) > 0) {

    timeNow = millis();
    timeDiff = timeNow - timeLast;
    timeLast = timeNow;

    double dps = ((currentAngle - lastAngle) / (timeDiff / 1000.0));
    int tempCurrentVelocity = ((int) degreesPerSecondToRPM(dps));
    if (tempCurrentVelocity < 80 && tempCurrentVelocity > -80) {
      currentVelocity = tempCurrentVelocity;
    }
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
  return ((dps * 60) / 360.0);
}

int MotorVelocityController::getAngle() {

  return (((*myEnc).read()/2) % 360);

}

