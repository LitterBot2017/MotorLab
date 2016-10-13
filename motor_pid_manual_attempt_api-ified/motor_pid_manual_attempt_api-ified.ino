#include "motor_position_controller.h"
#include "motor_velocity_controller.h"

// Pins
const byte motorEncoderAPort = 2;
const byte motorEncoderBPort = 3;
const byte motorL1Port = 5;
const byte motorL2Port = 6;
const byte motorSpeedPort = 11;

int desired;

MotorController motorController(motorL1Port, motorL2Port, motorSpeedPort);
MotorPositionController* motorPositionController;
MotorVelocityController* motorVelocityController;
Encoder encoder(motorEncoderAPort, motorEncoderBPort);

void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  motorPositionController = new MotorPositionController(&encoder, motorL1Port, motorL2Port, motorSpeedPort, &motorController);
  motorVelocityController = new MotorVelocityController(&encoder, motorL1Port, motorL2Port, motorSpeedPort, &motorController);

  desired = 0;
}

void loop() {

  if (Serial.available() > 0) {
    int serialInput = Serial.parseInt();
    if (serialInput != 0) {
      Serial.println("desiredAngle = " + String(serialInput));
      desired = (serialInput % 360);
    }
  }
  //Serial.println("currentAngle: " + String(motorPositionController.getAngle()));

  (*motorVelocityController).setVelocity(desired);

  //(*motorPositionController).setAngle(desired);

}

