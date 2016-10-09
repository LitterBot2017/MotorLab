#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "Arduino.h"

class Thermistor
{
public:

       Thermistor(int pin);
       int gettemperature();
private:
        int pinNum;
};
#endif