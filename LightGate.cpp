/******** Light Gate API cpp file ************/

#include "Arduino.h"
#include "Morse.h"

LightGate::LightGate(int pin){
	pin_num = pin;
}

int 
LightGate::getState(){
	return digitalRead(pin);
}