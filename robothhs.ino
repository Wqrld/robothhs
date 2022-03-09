#include <Servo.h>

//////////////////
// Docs voor AFMOTOR
// https://codebender.cc/library/AFMotor#AFMotor.cpp
/////////////////


// Wheel definitions
#define frontleft 1
#define frontright 2
#define backleft 3
#define backright 4

// Sensors
#define distanceSensorIn 5
#define distanceSensorOut 6
#define ledSensor 7 

// threshold voor hoe goed we de led moeten kunnen zien om te gaan rijden
#define threshold 50

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

// Arduino pin names for interface to 74HCT595 latch
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

/////////////
// back enabled
/////////////

// Constants that the user passes in to the motor calls
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// Servo
#define servoPin 8
Servo s;

static uint8_t latch_state;

void drive(uint8_t cmd, uint8_t motornum) {
  uint8_t a, b;
  switch (motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    latch_tx();
    break;
  }
}

void latch_tx(void) {
  uint8_t i;

  //LATCH_PORT &= ~_BV(LATCH);
  digitalWrite(MOTORLATCH, LOW);

  //SER_PORT &= ~_BV(SER);
  digitalWrite(MOTORDATA, LOW);

  for (i=0; i<8; i++) {
    //CLK_PORT &= ~_BV(CLK);
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & _BV(7-i)) {
      //SER_PORT |= _BV(SER);
      digitalWrite(MOTORDATA, HIGH);
    } else {
      //SER_PORT &= ~_BV(SER);
      digitalWrite(MOTORDATA, LOW);
    }
    //CLK_PORT |= _BV(CLK);
    digitalWrite(MOTORCLK, HIGH);
  }
  //LATCH_PORT |= _BV(LATCH);
  digitalWrite(MOTORLATCH, HIGH);
}

void setup() {
  // pinmodes wheels
  pinMode(frontleft, OUTPUT);
  pinMode(frontright, OUTPUT);
  pinMode(backleft, OUTPUT);
  pinMode(backright, OUTPUT);

// pinmodes sensors
pinMode(distanceSensorOut, OUTPUT);
pinMode(distanceSensorIn, INPUT);
pinMode(ledSensor, INPUT);

// Servo
s.attach(servoPin);

// Motorcontroller
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  latch_state = 0;

  latch_tx();

// Enable motor
  digitalWrite(MOTORENABLE, LOW);
  

}

int getDistance() {

// Send out 10microsecond pulse
  digitalWrite(distanceSensorOut, HIGH);
  delayMicroseconds(10);
  digitalWrite (distanceSensorOut,LOW); 

  uint32_t duration = pulseIn(distanceSensorIn, HIGH); // Find rtt duration

  double distance = duration *0.034029/2; // Calculate distance
  return distance;
}

int getIRBrightness(){
  // Todo implement

  return 0;
}

void loop() {
int servoAngle = 0;
s.write(servoAngle);
int hoogsteBrightness = 0;


while(true){
int hoogsteAngle = 0;
int hoogsteBrightness = 0;
for(int i = 0; i <= 18; i++){
  s.write(10 * i);
  int brightness = getIRBrightness();
  if(brightness > hoogsteBrightness){
    hoogsteBrightness = brightness;
    hoogsteAngle = 10 * i;
  }
}
//optimization: we kunnen al exiten voor we de hele loop door zijn als het weer minder wordt:

if(hoogsteBrightness > threshold){
// We hebben nu de hoogste brightness en deze was meer dan de threshold, rijden
  break;
}
// Als we hier aankomen dan konden we niks vinden, draaien en nog eens de hele dans doen

}

//todo: draai in de goede richting en rijd naar voren
//rij_voren(500)



// we komen steeds dichterbij, herhaal de loop tot we er zijn.

}
