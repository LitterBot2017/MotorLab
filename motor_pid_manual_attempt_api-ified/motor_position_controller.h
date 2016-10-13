#ifndef MOTOR_POSITION_CONTROLLER_H
#define MOTOR_POSITION_CONTROLLER_H

#include "Arduino.h"
#include "motor_controller.h"
#include <Encoder.h>

// Two public functions:
// setMotorPosition (degrees)
// getMotorAngle (degrees)

class MotorPositionController {

  public:
    MotorPositionController(byte pMotorEncoderAPort, byte pMotorEncoderBPort, byte pMotorL1Port, byte pMotorL2Port, byte pMotorSpeedPort, MotorController* pMotorController);
    int getAngle();
    void setAngle(int desired);

  private:
    // Encoder setup
    Encoder* myEnc;

    // Motor controller
    MotorController* motorController;

    // PID Control Setup
    int currentAngle;
    int desiredAngle;
    
    int errorNow;
    int errorLast;
    int errorDiff;
    long errorSum;
    
    int timeNow;
    int timeLast;
    int timeDiff;
    
    int pControl;
    int iControl;
    int dControl;
    int control;
  
    byte mMotorEncoderAPort;
    byte mMotorEncoderBPort;
    byte mMotorL1Port;
    byte mMotorL2Port;
    byte mMotorSpeedPort;

    const double kp = 5, ki = 0.001, kd = 0;
};

#endif
