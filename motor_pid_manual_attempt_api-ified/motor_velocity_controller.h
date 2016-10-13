#ifndef MOTOR_VELOCITY_CONTROLLER_H
#define MOTOR_VELOCITY_CONTROLLER_H

#include "Arduino.h"
#include "motor_controller.h"
#include "motor_position_controller.h"
#include <Encoder.h>

// Two public functions:
// setMotorVelocity (rpm)
// getMotorSpeed (rpm)

class MotorVelocityController {

  public:
    MotorVelocityController(byte pMotorEncoderAPort, byte pMotorEncoderBPort, byte pMotorL1Port, byte pMotorL2Port, byte pMotorSpeedPort, MotorController* pMotorController);
    int getVeloctiyRPM();
    void setVelocity(int desiredRPM);

  private:
    // Encoder setup
    Encoder* myEnc;

    // Motor controller
    MotorController* motorController;

    // PID Control Setup
    int currentVelocity;
    int desiredVelocity;

    int currentAngle;
    int lastAngle;

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
  
    byte mMotorEncoderAPort;
    byte mMotorEncoderBPort;
    byte mMotorL1Port;
    byte mMotorL2Port;
    byte mMotorSpeedPort;

    const double kp = 0.2, ki = 0, kd = 0.02;

    double degreesPerSecondToRPM(double dps);
    int getAngle();
};

#endif
