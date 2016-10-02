/******** Light Gate API ************/
#ifndef LIGHTGATE_H
#define LIGHTGATE_H

#include "Arduino.h"

class LightGate
{
public:
	LightGate(int pin);
	int getState();
private:
	int pin_num;
};

#endif
