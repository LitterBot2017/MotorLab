#include "Arduino.h"
#include "IRSensor.h"

IRSensor::IRSensor(int pin)
{
    pinNum=pin;
}

int IRSensor::distanceReading()
{
   int analogIn=analogRead(pinNum);
   return analogIn;
}
