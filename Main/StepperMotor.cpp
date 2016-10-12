#include "Arduino.h"
#include "StepperMotor.h"

StepperMotor::StepperMotor(int _dirpin, int _steppin)
{
    dirPin=_dirpin;
    pinMode(dirPin,OUTPUT);
	  stepPin=_steppin;
	  pinMode(stepPin,OUTPUT);
	
}

void StepperMotor::stepDegrees(bool dir, int steps)
{
 digitalWrite(dirPin,dir);
 delay(50);
 for(int i=0;i<steps;i++){
   digitalWrite(stepPin, HIGH);
   delayMicroseconds(800);
   digitalWrite(stepPin, LOW);
   delayMicroseconds(800);
}
}

void StepperMotor::moveDegrees(int degreeNumber)
{
  if (degreeNumber > 0){
    userAbs = (degreeNumber);
    stepNumber = (userAbs * 200/360);
    stepDegrees(true,stepNumber);
 }
  if (degreeNumber < 0){
    userAbs = (-1*degreeNumber);
    stepNumber = (userAbs * 200/360);
    stepDegrees(false,stepNumber);
  }
}
