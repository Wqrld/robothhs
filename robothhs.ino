#include <Servo.h>

//todo trigger pin echo pin distout distin
// linksachter motor2
// linksvoor M1
// rechtsvoor m4
// rechtsachter m3

bool manual = 0;

#define LA 2
#define LV 1
#define RV 4
#define RA 3

//////////////////
// Docs voor AFMOTOR
// https://codebender.cc/library/AFMotor#AFMotor.cpp
//
// #define _BV(bit) (1 << (bit))
// #define bit(b) (1UL << (b))
//
/////////////////

// Sensors
#define EchoPin A0 // are these defined correctly? probably want to switch around
#define TriggerPin A1

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

// Constants that the user passes in to the motor calls
#define FORWARD 1
#define BACKWARD 2
#define RELEASE 4

#define LEFT 5
#define RIGHT 6
#define TurnLeft 7
#define TurnRight 8

// Motor frequency definitions (all 1khz, may be audible but best torque)
#define MOTOR12_1KHZ _BV(CS22)              // divide by 64
#define MOTOR34_1KHZ _BV(CS01) | _BV(CS00)  // divide by 64

// Servo
#define servoPin 10
Servo s;


// Stored the state of the shift register
static uint8_t latch_state;

// Poort indices voor de IR baken sensoren.
int Irbakken[] = {A2, A3, A4, A5};
//v,l,a,r


/******************************************
               MOTORS
******************************************/

// Timer registers:
// TCCR: timer control register, zegmaar alle settings
// OCR: output compare register, voor de duty cycle

// VGM20 & VGM21 bits: waveform generation. WGM20 en WGM21 op op 0b11 geven aan om fast pwm te gebruiken.
// Alternatief is phase correct pwm, zie https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm
// COMn* bits: Compare on match, enable/disable/invert output n
// in ons geval zetten we ze aan op non-inverted.

inline void initPWM1(uint8_t freq) {

  // use PWM from timer2A on PB3 (Arduino pin #11)
  TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
  TCCR2B = freq & 0x7;

  // Duty cycle op 0
  OCR2A = 0;


  pinMode(11, OUTPUT);

}

inline void initPWM2(uint8_t freq) {

  // use PWM from timer2B (pin 3)
  TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
  TCCR2B = freq & 0x7;

  // Duty cycle op 0
  OCR2B = 0;


  pinMode(3, OUTPUT);
}


inline void initPWM3(uint8_t freq) {

  // use PWM from timer0A / PD6 (pin 6)
  TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on OC0A
  //TCCR0B = freq & 0x7;

  // Duty cycle op 0
  OCR0A = 0;

  pinMode(6, OUTPUT);
}

inline void initPWM4(uint8_t freq) {

  // use PWM from timer0B / PD5 (pin 5)
  TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0a
  //TCCR0B = freq & 0x7;

  // Duty cycle op 0
  OCR0B = 0;

  pinMode(5, OUTPUT);
}

// Set duty cycles
void setSpeed(uint8_t s) {
  // OCR = output compare register
  // Je timer telt op tot 255, en gaat uit als de waarde van het OCR* register gehaald is.
  // Hierdoor krijg je een duty cycle op een schaal van 0-255

  // motor 1, use PWM from timer2A on PB3 (Arduino pin #11)
  OCR2A = s;

  // motor 2, use PWM from timer2B (pin 3)
  OCR2B = s;

  // motor 3, use PWM from timer0A / PD6 (pin 6)
  OCR0A = s;

  // motor 4, use PWM from timer0B / PD5 (pin 5)
  OCR0B = s;
}


// zet alle motoren op 0 en stel de timers in.
void initMotor(uint8_t num, uint8_t freq) {

  switch (num) {
    case 1:
      latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
      latch_tx();
      initPWM1(freq);
      break;
    case 2:
      latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
      latch_tx();
      initPWM2(freq);
      break;
    case 3:
      latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
      latch_tx();
      initPWM3(freq);
      break;
    case 4:
      latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
      latch_tx();
      initPWM4(freq);
      break;
  }
}

// een bepaalde richting op rijden. CMD: FORWARD, BACKWARD, RELEASE, LEFT, RIGHT, TurnLeft, TurnRight
void driveDirection(uint8_t cmd) {
  if (cmd == FORWARD || cmd == BACKWARD || cmd == RELEASE) {
    // We can just directly forward this to all our motors
    drive(cmd, LV);
    drive(cmd, LA);
    drive(cmd, RV);
    drive(cmd, RA);
  } else if (cmd == LEFT) {
    drive(FORWARD, LA);
    drive(BACKWARD, RA);
    drive(BACKWARD, LV);
    drive(FORWARD, RV);
  } else if (cmd == RIGHT) {
    drive(FORWARD, LA);
    drive(FORWARD, RA);
    drive(FORWARD, LV);
    drive(FORWARD, RV);
  } else if (cmd == TurnRight) {
    drive(FORWARD, LV);
    drive(FORWARD, LA);
    drive(BACKWARD, RV);
    drive(BACKWARD, RA);
  } else if (cmd == TurnLeft) {
    drive(BACKWARD, LV);
    drive(BACKWARD, LA);
    drive(FORWARD, RV);
    drive(FORWARD, RA);
  }
}

//Stuur 1 motor aan. Cmd: FORWARD, BACKWARD, of RELEASE
void drive(uint8_t cmd, uint8_t motornum) {
  uint8_t a, b;
  if ((motornum == 2 || motornum == 3)) {

    if (cmd == FORWARD) {
      cmd = BACKWARD;
    } else if (cmd == BACKWARD) {
      cmd = FORWARD;
    }

  }

  //Decide which motor we should use
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
      latch_state |= _BV(a); // Set bit a to 1
      latch_state &= ~_BV(b);  //Set bit b to 0
      latch_tx();
      break;
    case BACKWARD:
      latch_state &= ~_BV(a); // Set bit a to 0
      latch_state |= _BV(b);  // Set bit b to 1
      latch_tx();
      break;
    case RELEASE:
      latch_state &= ~_BV(a);     // A and B both low (0)
      latch_state &= ~_BV(b);
      latch_tx();
      break;
  }
}


// Verzend de latch status naar het shift register.
void latch_tx(void) {
  uint8_t i;

  //LATCH_PORT &= ~_BV(LATCH);
  digitalWrite(MOTORLATCH, LOW);

  //SER_PORT &= ~_BV(SER);
  digitalWrite(MOTORDATA, LOW);

  for (i = 0; i < 8; i++) {
    //CLK_PORT &= ~_BV(CLK);
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & _BV(7 - i)) {
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

  // pinmodes sensors
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // Servo
  s.attach(servoPin);

  // Motorcontroller
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  latch_state = 0; // reset all motor states
  latch_tx();

  // Enable motor
  digitalWrite(MOTORENABLE, LOW);

  //1khz geeft de beste efficiency
  initMotor(1, MOTOR12_1KHZ);
  initMotor(2, MOTOR12_1KHZ);
  initMotor(3, MOTOR34_1KHZ);
  initMotor(4, MOTOR34_1KHZ);

  // speed (duty cycle) up to 255, maar tot 128 veilig
  setSpeed(128);

  //Possible options: FORWARD, BACKWARD, RELEASE, LEFT, RIGHT, TurnLeft, TurnRight
  driveDirection(TurnLeft);
  // Init serial output
  Serial.begin(9600);
  Serial.flush();

}

unsigned long getDistance() {

  // Send out 10microsecond pulse
  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite (TriggerPin, LOW);

  //TODO add timeout, in microseconds
  unsigned long duration = pulseIn(EchoPin, HIGH); // Find rtt duration

  unsigned long distance = duration * 0.034029 / 2; // Calculate distance
  return distance;
}

// Run this from the main loop, HHS-11
void checkBlueTooth() {
  while (Serial.available() > 0) {

    char input = (char) Serial.read();
    // pagina 32 voor de benodigde opdrachten, vb:

    // char, dus enkele aanhalingstekens
    switch(input){
      case 'F':
        driveDirection(FORWARD);
        break;
      case 'B':
        driveDirection(BACKWARD);
        break;
      case 'L':
        driveDirection(TurnLeft);
        break;
      case 'R':
        driveDirection(TurnRight);
        break;
      case 'S':
        driveDirection(RELEASE);
        break;
      case '+':
        break;
      case '-':
        break;
      case 'A':
        manual = 0;
        break;
      case 'C':
        manual = 1;
        break;
      
    }
      

    // ....


  }
}
  int TurnTries = 0;
void loop() {
  // Reset alle info voor een nieuwe run

  int hoogsteBrightness = 0;
int servoAnglelinks = 15;
  int servoAngleRechtdoor = 90;
  int servoAngleRechts = 165;
  int readarray[5];
  int maxVal = 0;
  int maxZ = 0;
  int SecondmaxZ = 0;
  int secndmaxVal = 0;
  //als afstand kleiner is dan 10 en linker IR Led de secondmaxvalue kant is en voor MaxValue is, dan kijken we met Distance sensor naar links en kijken we of er een obstakel is
 if (distance < 10 && SecondmaxZ == 1 && maxZ == 0) {
   driveDirection(RELEASE);
   delay(100);
   s.write(servoAnglelinks);
   if (distance < 10) {
    s.write(servoAngleRechts);
    if ( distance > 10 ) {
        //Als links een obstakel is en rechts niet dan gaan we naar rechts staan, dan voren, dan links staan, dan rechtdoor
  
   driveDirection(TurnRight);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   s.write(servoAngleRechtdoor);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(TurnLeft);
   delay(150);
    driveDirection(RELEASE);
   delay(100);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   } 
   //als de afstand van rechterkant ook kleiner is dan 10, dan rijdt het wagen even naar achter en gaat dan naar links
   else {
     driveDirection(BACKWARD);
     delay(150);
     driveDirection(RELEASE);
     delay(100);
     driveDirection(TurnLeft);
     delay(100);
     driveDirection(RELEASE);
     delay(100);
     driveDirection(FORWARD);
     delay(100);
     driveDirection(RELEASE);
     delay(100);
   }
    } 
    //als afstand links groter is dan 10 dan rijden we naar links en dan rijden we naar rechts om obstakel te vermijden
    else {
   driveDirection(TurnLeft);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(TurnRight);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   }
 }
//als afstand kleiner is dan 10 en rechter IR Led de secondmaxvalue kant is en voor MaxValue is, dan kijken we met Distance sensor naar rechts en kijken we of er een obstakel is
 if (distance < 10 && SecondmaxZ == 3 && maxZ == 0) {
  driveDirection(RELEASE);
  delay(100);
  s.write(servoAngleRechts);
  if (distance < 10) {
    s.write(servoAnglelinks);
    if ( distance > 10 ) {
        //Als rechts een obstakel is en links niet dan gaan we naar links staan, dan voren, dan rechts staan, dan rechtdoor
      
   driveDirection(TurnLeft);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   s.write(servoAngleRechtdoor);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(TurnRight);
   delay(150);
    driveDirection(RELEASE);
   delay(100);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   } 
   //als de afstand van linkerkant ook kleiner is dan 10, dan rijdt het wagen even naar achter en gaat dan naar rechts
   else {
     driveDirection(BACKWARD);
     delay(150);
     driveDirection(RELEASE);
     delay(100);
     driveDirection(TurnRight);
     delay(100);
     driveDirection(RELEASE);
     delay(100);
     driveDirection(FORWARD);
     delay(100);
     driveDirection(RELEASE);
     delay(100);
   }
    } 
    //als de afstand rechts groter is dan 10 dan rijden we naar rechts
  else{
   driveDirection(TurnRight);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(TurnLeft);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   driveDirection(FORWARD);
   delay(150);
   driveDirection(RELEASE);
   delay(100);
   }
 }



  // Hebben we nieuwe bluetooth commandos gehad?
  checkBlueTooth();
  if(manual){
    return; // If manual, don't run through the algorithm.
  }

  int hoogsteAngle = 0;
  hoogsteBrightness = 0;

  // HCSR04 distance sensor afstand
  //Serial.println(getDistance());


  // Hier kunnen we onze waarden in opslaan
  int readarray[5];
  int maxVal = 0;
  int maxZ = 0;
  int secndmaxVal = 0;
  
  // Kort stoppen om te meten
  driveDirection(RELEASE);
  delay(100); // Genoeg om zeker te meten, maar toch snel

  /*
   * Als hoger dan vorige hoogste
   * 2ndhoogste = vorige hoogste
   * 
   * 
   */
  
  for (int z = 0; z < 4; z++) {
    // doe een lezing en sla deze op
    readarray[z] = analogRead(Irbakken[z]);
    Serial.print(z);
    Serial.print("-");
    Serial.println(readarray[z]);

    // Is dit de hoogste waarde?
    if (readarray[z] > maxVal) {

      secndmaxVal = maxVal;
      maxVal = readarray[z];
      maxZ = z;
    }
    if (readarray [z] < maxVal){
      for (int SK= 0; SK < 4; SK++){ 
        if (nieuw < readarray[z] && nieuw != maxVal){
        nieuw = readarry[z]}
           }
        }
      }
    
  }

      // Afstand meten, nog niet in gebruik op het moment
   // Serial.print("HCSR04 distance: ");
    //Serial.println(getDistance());

  // Is onze hoogste meting meer dan de threshold van 120?
  // Heeft flink wat tuning nodig, de range stelt weinig voor.
  if (maxVal > secndmaxVal + 15) {
    //RIJ NAAR RICHTING Z
    // v,l,a,r
    if (maxZ == 0) {
      driveDirection(FORWARD);
      delay(700);
    } else if (maxZ == 1) {
      driveDirection(TurnLeft);
      delay(300);
    } else if (maxZ == 2) {
      driveDirection(BACKWARD);
      delay(700);
    } else if (maxZ == 3) {
      driveDirection(TurnRight);
      delay(300);
    }
    driveDirection(RELEASE);

  } else {
    // Niks gevonden, draaien en nog eens proberen.
    driveDirection(TurnLeft);
    delay(150);
    driveDirection(RELEASE);
    TurnTries++;
  }

  if (TurnTries > 10) {
    // check if something in front of us?
    // drive forward if not as we cant find shit
    // and retry everything

    int counter = 0;
    while(getDistance() > 10 && counter < 5){
      driveDirection(FORWARD);
      delay(200);
      driveDirection(RELEASE);
      delay(50); // kort stoppen zodat we stilstaan voor de afstand meting
      counter++;
    }

    TurnTries = 0;
  }

}
