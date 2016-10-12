const byte motorEncoderAPort = 2;
const byte motorEncoderBPort = 3;
const byte motorL1Port = 5;
const byte motorL2Port = 6;
const byte motorEnablePort = 11;

float angle = 0.0;
int phase;

int encoderAResult;
int encoderBResult;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  analogWrite(motorEnablePort, 105);
  digitalWrite(motorL1Port, 0);
  digitalWrite(motorL2Port, 1);

  pinMode(motorEncoderAPort, INPUT);
  pinMode(motorEncoderBPort, INPUT);

  attachInterrupt(digitalPinToInterrupt(motorEncoderAPort), encoderAUpdated, RISING);
  attachInterrupt(digitalPinToInterrupt(motorEncoderBPort), encoderBUpdated, RISING);
}

void loop() {
  
  //Serial.println("A = " + String(digitalRead(motorEncoderAPort)));
  //Serial.println("B = " + String(digitalRead(motorEncoderBPort)));
  
}

void encoderAUpdated() {
  Serial.println("A running.");
  encoderAResult = digitalRead(motorEncoderAPort);
  updateAngle();
}

void encoderBUpdated() {
  Serial.println("B running.");
  encoderBResult = digitalRead(motorEncoderBPort);
  updateAngle();
}

void updateAngle() {

  // If the phase hasn't been defined...
  if (!phase) {
    setPhase(encoderAResult, encoderBResult);
  }
  
  else {
    int oldPhase = phase;
    setPhase(encoderAResult, encoderBResult);
    //setAngle(phase, oldPhase);
  }
}

void setPhase(int encoderA, int encoderB) {
  phase = (encoderA << 1) + encoderB;
  Serial.println(String(encoderA) + String(encoderB));
  Serial.println("Phase = " + String(phase));
}

void setAngle(int newPhase, int oldPhase) {
  int total = (newPhase << 2) + oldPhase;
  switch (total) {
    // CCW from phase 01 to phase 00
    case 1:
    // CCW from phase 00 to phase 10
    case 8:
    // CCW from phase 10 to phase 11
    case 15:
    // CCW from phase 11 to phase 01
    case 7:
      angle -= 2;
      break;

    // CW from phase 00 to phase 01
    case 4:
    // CW from phase 01 to phase 11
    case 13:
    // CW from phase 11 to phase 10
    case 11:
    // CW from phase 10 to phase 00
    case 2:
      angle += 2;
      break;

    default:
      Serial.println("Motor encoder entered illegal phase transition! oldPhase: " + String(oldPhase) + "; newPhase: " + String(newPhase));
    
  }
}


