#include "Arduino.h"
#include "StepperMotor.h"

StepperMotor::StepperMotor(int pin1, int pin2)
{
    dirPin=pin1;
    pinMode(dirPin,OUTPUT);
	  stepPin=pin2;
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
