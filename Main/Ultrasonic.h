#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "Arduino.h"

class Ultrasonic
{
  public:
    Ultrasonic(int pin);
	  int filteredReading();
    int pulse_width_measurement();
  private:
	  int pinNum;
};

#endif

