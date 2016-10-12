#include"Arduino.h"
#include"Thermistor.h"

Thermistor::Thermistor(int pin)
{

      pinNum= pin;
}

int Thermistor::gettemperature()
{
    int thermistorReading = analogRead(pinNum);
    return thermistorReading;
}
