/******** Light Gate API cpp file ************/

#include "Arduino.h"
#include "LightGate.h"

LightGate::LightGate(int pin){
	pin_num = pin;
	pinMode(pin, INPUT);
}

int LightGate::getState(){
	int state=digitalRead(pin_num);
	return state;
}
