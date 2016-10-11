#include "Arduino.h"

StepperMotor::StepperMotor(int pin1, int pin2)
{
    dirPin=pin1;
    pinMode(dirPin,OUTPUT);
	stepperPin=pin2;
	pinMode(stepperPin,OUTPUT)
	
}

int StepperMotor::step(bool dir, int steps);
{
 digitalWrite(dirPin,dir);
 delay(50);
 for(int i=0;i<steps;i++){
   digitalWrite(stepperPin, HIGH);
   delayMicroseconds(800);
   digitalWrite(stepperPin, LOW);
   delayMicroseconds(800);
}

int StepperMotor::moveDegrees(int degrees);
{
  if (degrees > 0){
    userAbs = (degrees);
    stepNumber = (userAbs * 200/360);
    step(true,stepNumber);
 }
  if (degrees < 0){
    userAbs = (-1*degrees);
    stepNumber = (userAbs * 200/360);
    step(false,stepNumber);
  }
}