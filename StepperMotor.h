#ifndef StepperMotor
#define StepperMotor

#include "Arduino.h"

class StepperMotor
{
    public:
	      StepperMotor(int pin1, int pin2);
          moveDegrees(int degrees);
    private:
         step(bool dir, int steps);
		 int dirPin;
		 int stepPin;
		 float userAbs;
		 float stepNumber;
};
#endif
boolean