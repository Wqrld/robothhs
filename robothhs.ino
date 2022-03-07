#include <Servo.h>

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

// Servo
#define servoPin 8
Servo s;


void setup() {
  // pinmodes wheels
  pinMode(frontleft, OUTPUT);
  pinMode(frontright, OUTPUT);
  pinMode(backleft, OUTPUT);
  pinMode(backright, OUTPUT);

// pinmodes sensors
pinMode(distanceSensorOut, OUTPUT);
pinMode(distanceSensorIn, INPUT);
pinMode(ledSensor, INPUT)

// Servo
s.attach(servoPin);
  

}

int getDistance() {

// Send out 10microsecond pulse
  digitalWrite(distanceSensorOut, HIGH);
  delayMicroseconds(10);
  digitalWrite (distanceSensorOut,LOW); 

  duration = pulseIn(distanceSensorIn, HIGH); // Find rtt duration

  distance = duration *0.034029/2; // Calculate distance
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
for(int i = 0; i <= 18; i++){
  s.write(10 * i);
  int brightness = getIRBrightness();
  if(brightness > hoogsteBirhgtness){
    hoogsteBirhgtness = brightness;
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
