#ifndef StepperMotor_h
#define StepperMotor_h

#include "Arduino.h"

class StepperMotor
{
    public:
	        StepperMotor(int _dirpin, int _steppin);
     void moveDegrees(int degreeNumber);
    private:
     void stepDegrees(bool dir, int steps);
		 int dirPin;
		 int stepPin;
		 float userAbs;
		 float stepNumber;
};
#endif
