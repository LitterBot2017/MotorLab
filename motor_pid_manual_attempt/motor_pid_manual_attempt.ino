#include <Encoder.h>

// Four functions:
// setMotorPosition (degrees)
// setMotorVelocity (rpm)
// getMotorAngle (degrees)
// getMotorSpeed (rpm)

// Pins
const byte motorEncoderAPort = 2;
const byte motorEncoderBPort = 3;
const byte motorL1Port = 5;
const byte motorL2Port = 6;
const byte motorSpeedPort = 11;

// Encoder setup
Encoder myEnc(2, 3);

// PID Control Setup
int currentAngle;
int velocity;
int desiredAngle;

int errorNow;
int errorLast;
int errorDiff;
int errorSum;

int timeNow;
int timeLast;
int timeDiff;

int pControl;
int iControl;
int dControl;
int control;

double kp = 0.2, ki = 0, kd = 0.02;

void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // Motor pin setup
  pinMode(motorL1Port, OUTPUT);
  pinMode(motorL2Port, OUTPUT);
  pinMode(motorSpeedPort, OUTPUT);

  // Motor Encoder Pin Setup
  pinMode(motorEncoderAPort, INPUT);
  pinMode(motorEncoderBPort, INPUT);

  timeLast = millis();
  errorLast = 0;
}

void loop() {

  currentAngle = ((myEnc.read()/2) % 360);

  if (Serial.available() > 0) {
    int serialInput = Serial.parseInt();
    if (serialInput != 0) {
      Serial.println("desiredAngle = " + String(serialInput));
      desiredAngle = (serialInput % 360);
    }
  }

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
    digitalWrite(motorL1Port, LOW);
    digitalWrite(motorL2Port, HIGH);
    analogWrite(motorSpeedPort, 0);
  } else {
    turnMotor(control);
  }
}

void turnMotor(int velocity) {
  if (velocity > 255) {
    velocity = 255;
  } else if (velocity < -255) {
    velocity = -255;
  }

  if (velocity > 0 && velocity < 90) {
    velocity = 90;
  } else if (velocity < 0 && velocity > -90) {
    velocity = -90;
  }

  if (abs(velocity) > 0) {
    if ((millis() % 1000) == 0) {
      Serial.println("currAng: " + String(currentAngle) + "; desAng: " + String(desiredAngle) + "; vel: " + String(velocity) + "; velocity: " + String(velocity));
    }

    if (velocity < 0) {
      digitalWrite(motorL1Port, HIGH);
      digitalWrite(motorL2Port, LOW);
    } else {
      digitalWrite(motorL1Port, LOW);
      digitalWrite(motorL2Port, HIGH);
    }
    analogWrite(motorSpeedPort, abs(velocity));
  }
}

