#include <PID_v1.h>
#include <Encoder.h>

// Pins
const byte motorEncoderAPort = 2;
const byte motorEncoderBPort = 3;
const byte motorL1Port = 5;
const byte motorL2Port = 6;
const byte motorSpeedPort = 11;

// Encoder setup
Encoder myEnc(2, 3);

// PID Control Setup

double currentAngle;
double velocity;
double setDesiredAngle;
double desiredAngle;

int errorNow;
int errorLast;
int errorDiff;

int timeNow;
int timeLast;
int timeDiff;

double kp = 0.05, ki = 0, kd = 0;

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
}

void loop() {

  currentAngle = (myEnc.read() / 2);

  if (Serial.available() > 0) {
    desiredAngle = Serial.parseInt();
    if (desiredAngle != 0) {
      Serial.println("desiredAngle = " + String(desiredAngle));
      setDesiredAngle = currentAngle + desiredAngle;
    }
  }

  
  errorNow = desiredAngle - currentAngle;
  timeNow = millis();
  timeDiff = timeNow - timeLast;
  errorDiff = errorNow - errorLast;

  
  turnMotor(velocity);
}

void turnMotor(int velocity) {
  if (abs(velocity) > 0) {
    Serial.println("currAng: " + String(currentAngle) + "; desAng: " + String(setDesiredAngle) + "; vel: " + String(velocity) + "; speed: " + String((abs(velocity) + 79)));
    
    if (velocity < 0) {
      digitalWrite(motorL1Port, HIGH);
      digitalWrite(motorL2Port, LOW);
    } else {
      digitalWrite(motorL1Port, LOW);
      digitalWrite(motorL2Port, HIGH);
    }
    analogWrite(motorSpeedPort, (abs(velocity) + 79));
  }
}

