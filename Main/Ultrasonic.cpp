#include "Arduino.h"
#include "Ultrasonic.h"

const int readingsNum = 5;

int filterWindow[readingsNum];
int total=0;
float average=0;
int reading=0;

Ultrasonic::Ultrasonic(int pin)
{
    pinNum=pin;
}

int Ultrasonic::filteredReading()
{
   int analogIn=analogRead(pinNum);
   total=total-filterWindow[reading];
   filterWindow[reading]=analogIn;
   total=total+filterWindow[reading];
   reading=reading+1;
   if(reading>=readingsNum)
   {
     reading=0;
   }
   return total/readingsNum;
}
    
