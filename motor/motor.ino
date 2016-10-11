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
double kp = 1.3, ki = 0.2, kd = 0.03;

// Input = currPosition, output = velocity, setpoint = speed
PID myPID(&currentAngle, &velocity, &setDesiredAngle, kp, ki, kd, DIRECT);

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

  myPID.SetMode(AUTOMATIC);

  myPID.SetOutputLimits(-255, 255);
}

void loop() {
  analogWrite(motorSpeedPort, 140);
    digitalWrite(motorL1Port, HIGH);
    digitalWrite(motorL2Port, LOW);
}

void turnMotor(int speed, int direction) {

  if (direction == -1) {
    Serial.println("Spinning counterclockwise " + String(speed));
    analogWrite(motorSpeedPort, speed);
    digitalWrite(motorL1Port, LOW);
    digitalWrite(motorL2Port, HIGH); 
  }

  else if (direction == 1) {
    Serial.println("Spinning clockwise " + String(speed));
    analogWrite(motorSpeedPort, speed);
    digitalWrite(motorL1Port, LOW);
    digitalWrite(motorL2Port, HIGH);
  }
}

