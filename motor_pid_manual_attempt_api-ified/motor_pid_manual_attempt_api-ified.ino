#include "motor_position_controller.h"
#include "motor_velocity_controller.h"

// Pins
const byte motorEncoderAPort = 2;
const byte motorEncoderBPort = 3;
const byte motorL1Port = 5;
const byte motorL2Port = 6;
const byte motorSpeedPort = 11;

int desiredAngle;

MotorController motorController(motorL1Port, motorL2Port, motorSpeedPort);
MotorPositionController* motorPositionController;
//MotorVelocityController* motorVelocityController;

void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  motorPositionController = new MotorPositionController(motorEncoderAPort, motorEncoderBPort, motorL1Port, motorL2Port, motorSpeedPort, &motorController);
  //motorVelocityController = new MotorVelocityController(motorEncoderAPort, motorEncoderBPort, motorL1Port, motorL2Port, motorSpeedPort, &motorController);
}

void loop() {

  if (Serial.available() > 0) {
    int serialInput = Serial.parseInt();
    if (serialInput != 0) {
      Serial.println("desiredAngle = " + String(serialInput));
      desiredAngle = (serialInput % 360);
    }
  }
  //Serial.println("currentAngle: " + String(motorPositionController.getAngle()));

  // Serial.println("velocityRPM: " + String((*motorVelocityController).getVeloctiyRPM()));

  (*motorPositionController).setAngle(desiredAngle);

}

