//initializing PIN constants
const int IN1 = 5; //two inputs for motor Driver L298N
const int IN2 = 6;
const int Encoder_ChA = 2;
const int Encoder_ChB = 3; // Encoder Pins
const int IN3 = 10; //PWM for motorspeed

int EncoderPos = 0; // encoder count
// Initializing constants for Position Feedback PID
int DesiredPos = 720;
int CurrentPos = 0;
int ErrorSum = 0;
int Timelast = 0;
int Errorlast = 0;
int Kp = 2.0;
int Ki = 0.01;
int Kd = 0.01;
//Initializing Constants for Velocity FeedBack PID
int Timelastv = 0;
int DesiredVel = 100*300/255;
int Errorsumv = 0;
int Errorlastv = 0;
int Kpv = 2;
int Kiv = 0.01;
int Kdv = 0.01;
void stopmotor();
void moveclockwise();
void moveanticlockwise();
void doEncoder();
void setup() {
 // put your setup code here, to run once:
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(Encoder_ChA, INPUT);
 digitalWrite(Encoder_ChA, HIGH); // turn on pullup resistor
 pinMode(Encoder_ChB, INPUT);
 digitalWrite(Encoder_ChB, HIGH); // turn on pullup resistor
 digitalWrite(IN3, 80);
 attachInterrupt(0, doEncoder, CHANGE); // encoder pin on interrupt 0 - pin 2
 Serial.begin (9600);
}
void loop()
{
 moveclockwise();
 //CurrentPos = EncoderPos;
 int PrevTime=0;
 int PrevPos=0;
 delay(3000);
 int Timenow=millis();
 int Vel= EncoderPos-PrevPos/(Timenow-PrevTime);
 while (DesiredVel> 0)
 {
 Serial.println(Vel);
 int Output = PIDVelControl(DesiredVel, Vel);
 moveclockwise;
 PrevTime= Timenow;
 PrevPos= EncoderPos;
 SetVelocity(Vel+Output);
 delay(1000);
 int Vel= EncoderPos-PrevPos/(millis()-PrevTime);
 }
 while (DesiredPos <0)
 {
 Serial.println(Vel);
 int Output = PIDVelControl(DesiredVel,Vel);
 moveanticlockwise;
 PrevTime= Timenow;
 PrevPos= EncoderPos;
 SetVelocity(Vel+Output);
 delay(1000);
 int Vel= EncoderPos-PrevPos/(millis()-PrevTime);
 }
 delay(200);
 Serial.println(EncoderPos);


}
void moveanticlockwise()
{
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
}
void moveclockwise()
{
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
}
void stopmotor()
{
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
}
//Function that activates on interrupts to check and calculate the encoder value.. Positive for clockwise and negative for anticlockwise
void doEncoder()
 {
 int aState = digitalRead(Encoder_ChA);
 int bState = digitalRead(Encoder_ChB);
 if (aState == HIGH) //Check state of Channel A
 {
 if (bState == LOW) //check state of Channel B
 {
 EncoderPos += 1; //Clockwise
 }
 else
 {
 EncoderPos -= 1; //CounterCLockwise
 }
 }
 else
 {
 if (bState == LOW)
 {
 EncoderPos -= 1; //CounterClockwise
 }
 else
 {
 EncoderPos += 1; //Clockwise
 }
 // delay(500);
 }
}
//Function to set MotorSpeed
void SetVelocity (int vel)
{
 analogWrite (IN3, vel);
}
void SetDirection (int dir)
{
 if (dir == 0)
 {
 moveclockwise();
 }
 else
 {
 moveanticlockwise();
 }
}
int MotorControl(int vel, int dir)
{
 SetVelocity(vel);
 SetDirection(dir);
}
// Implementing PID control on position
int PIDcontrol(int DesiredPos, int CurrentPos)
{
 int Error = DesiredPos - CurrentPos;
 int Timenow = millis();
 int TimeDiff = Timenow - Timelast;
 int ErrorDiff = Error - Errorlast;
 int PControl = Kp * Error;
 ErrorSum += (Error * TimeDiff);
 int IControl = Ki * ErrorSum;
 int DControl = Kd * ErrorDiff / TimeDiff;
 int Control = PControl + IControl + DControl;
 Timelast = Timenow;
 Errorlast = Error;
 return Control;
}
//Implementing PID control for Motor speed
int PIDVelControl(int DesiredVel, int CurrentVel)
{
 int Errorv = DesiredVel - CurrentVel;
 int Timenowv = millis();
 int TimeDiffv = Timenowv - Timelastv;
 int ErrorDiffv = Errorv - Errorlastv;
 int PControlv = Kpv * Errorv;
 Errorsumv += Errorsumv * TimeDiffv;
 int IControlv = Kiv * Errorsumv;
 int DControlv = Kdv * ErrorDiffv / TimeDiffv;
 int Controlv = PControlv + IControlv + DControlv;
 Timelastv = Timenowv;
 Errorlastv = Errorv;
 return Controlv;
}
void PIDposition(int DesiredPos)
{
 //CurrentPos = EncoderPos;
 stopmotor();
 while (DesiredPos - EncoderPos > 0)
{
 int Output = PIDcontrol(DesiredPos, EncoderPos);
 Serial.println(DesiredPos - EncoderPos);
 if (Output > 0)
 {
 if (Output > 255)
 {
 SetVelocity(255);
 moveclockwise();
 }
 else
 {
 SetVelocity(Output);
 }
 }
 }
 while (EncoderPos - DesiredPos > 0)
 {
 int Output = PIDcontrol(DesiredPos-110, EncoderPos);
 if (Output < -255)
 {
 SetVelocity(255);
 moveanticlockwise();
 }
 else
 {
 SetVelocity(Output);
 moveanticlockwise();
 }
 }
 delay(200);
 stopmotor();
 Serial.println(EncoderPos);
}
