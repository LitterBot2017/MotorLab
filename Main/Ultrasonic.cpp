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
    pinMode(pin,INPUT);
}

int Ultrasonic::pulse_width_measurement()
{  
   long pulse_val= pulseIn(pinNum,HIGH);
   if(pulse_val>6000)
   {
    pulse_val=6000;
   }
   total=total-filterWindow[reading];
   filterWindow[reading]=pulse_val;
   total=total+filterWindow[reading];
   reading=reading+1;
   if(reading>=readingsNum)
   {
     reading=0;
   } 
   return (int) total/readingsNum;
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
    
