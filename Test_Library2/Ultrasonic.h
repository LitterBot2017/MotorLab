#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "Arduino.h"

class Ultrasonic
{
    public:
	Ultrasonic(int pin);
	int filteredReading();
    private:
	int pinNum;
};

#endif

