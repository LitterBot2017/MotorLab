#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Arduino.h"
#include <Encoder.h>

// One public functions:
// setMotorPosition (degrees)
// getMotorAngle (degrees)

class MotorController {

  public:
    MotorController(byte pMotorEncoderAPort, byte pMotorEncoderBPort, byte pMotorL1Port, byte pMotorL2Port, byte pMotorSpeedPort);
    void turnMotor(int velocity);

  private:
    byte mMotorEncoderAPort;
    byte mMotorEncoderBPort;
    byte mMotorL1Port;
    byte mMotorL2Port;
    byte mMotorSpeedPort;
};

#endif

