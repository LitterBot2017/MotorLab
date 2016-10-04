#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Arduino.h"

class IRSensor
{
    public:
	      IRSensor(int pin);
        int distanceReading();
    private:
        int pinNum;
};
#endif
