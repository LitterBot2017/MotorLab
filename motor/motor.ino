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
double kp = 0.1, ki = 0, kd = 0.05;

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

  currentAngle = (myEnc.read() / 2);

  if (Serial.available() > 0) {
    desiredAngle = Serial.read();
    if (desiredAngle != 0) {
      setDesiredAngle = currentAngle + desiredAngle;
    }
  }

  myPID.Compute();

  turnMotor(velocity);
}

void turnMotor(int velocity) {
  if (abs(velocity) > 0) {
    Serial.println("currAng: " + String(currentAngle) + "; desAng: " + String(setDesiredAngle) + "; speed: " + String(((abs(velocity) + 80) % 256)));
    analogWrite(motorSpeedPort, ((abs(velocity) + 80) % 256));
    if (velocity < 0) {
      digitalWrite(motorL1Port, HIGH);
      digitalWrite(motorL2Port, LOW);
    } else {
      digitalWrite(motorL1Port, LOW);
      digitalWrite(motorL2Port, HIGH);
    }
  }
}

