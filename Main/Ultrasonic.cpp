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
   int pulse_val= (int) pulseIn(pinNum,HIGH);
   if(pulse_val<0)
   {
    pulse_val=3000;
   }
   total=total-filterWindow[reading];
   filterWindow[reading]=pulse_val;
   total=total+filterWindow[reading];
   reading=reading+1;
   if(reading>=readingsNum)
   {
     reading=0;
   } 
   return total/readingsNum;
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
    
