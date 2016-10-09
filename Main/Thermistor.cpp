#include"Arduino.h"
#include"Thermistor.h"

Thermistor::Thermistor(int pin)
{

      pinNum= pin;
      pinMode(pin,INPUT);
}

int Thermistor::gettemperature()
{
    int thermistorReading = analogRead(thermistorPin);
    return thermistorReading;
}