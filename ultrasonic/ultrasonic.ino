#include <ros.h>
#include <std_msgs/Float32.h>

const int readingsNum = 5;

ros::NodeHandle ultrasonic;
std_msgs::Float32 float_msg;
ros::Publisher distance("distance",&float_msg);

int filterWindow[readingsNum];
int total=0;
float average=0;
int reading=0;

void setup() {
  // put your setup code here, to run once:
  ultrasonic.initNode();
  ultrasonic.advertise(distance);
  pinMode(A0,INPUT);
  for(int i=0;i<readingsNum;i++)
  {
    filterWindow[i]=0;
  }
}

float filteredReading(int analogIn)
{
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

void loop() {
  // put your main code here, to run repeatedly:
  int measured_dist=analogRead(A0);
  average=filteredReading(measured_dist);
  float_msg.data=average;
  distance.publish(&float_msg);
  ultrasonic.spinOnce();
  delay(100);
}
