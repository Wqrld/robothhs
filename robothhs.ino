#include <Servo.h>

//todo trigger pin echo pin distout distin
// linksachter motor2
// linksvoor M1
// rechtsvoor m4
// rechtsachter m3

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
#define ledSensor 9 // todo: setup and probably change port

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

// Motor frequency definitions (all 1khz, may be audible but best torque)
#define MOTOR12_1KHZ _BV(CS22)              // divide by 64
#define MOTOR34_1KHZ _BV(CS01) | _BV(CS00)  // divide by 64

// Servo
#define servoPin 10
Servo s;


// Stored the state of the shift register
static uint8_t latch_state;

// Poort indices voor de IR baken sensoren.
int Irbakken[] = {A2, A5, A3, A4};


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

// TODO evt aanpassen voor de omgedraaide motoren?

void drive(uint8_t cmd, uint8_t motornum) {
  uint8_t a, b;

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

void latch_tx_short_untested(void) {
  // latch has to go low before transmitting
  digitalWrite(MOTORLATCH, LOW);
  // probably unneeded
  digitalWrite(MOTORDATA, LOW);
  // shift out the data, LSB first (untested, but matches the old latchtx)
  shiftOut(MOTORDATA, MOTORCLK, LSBFIRST, latch_state);
  // enable the latch again
  digitalWrite(MOTORLATCH, HIGH);
}

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
  pinMode(ledSensor, INPUT);

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

  //Possible options: FORWARD, BACKWARD, RELEASE 
  drive(FORWARD, 1); 
  drive(BACKWARD, 2);
  drive(BACKWARD, 3);
  drive(FORWARD, 4);
  
  // Init serial output
  Serial.begin(9600);
  Serial.flush();

}

unsigned long getDistance() {

  // Send out 10microsecond pulse
  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite (TriggerPin, LOW);

  unsigned long duration = pulseIn(EchoPin, HIGH); // Find rtt duration

  unsigned long distance = duration * 0.034029 / 2; // Calculate distance
  return distance;
}

int getIRBrightness() {
  // Todo implement

  return 0;
}

// Run this from the main loop
void checkBlueTooth() {
  while(Serial.available() > 0){
   
    char input = (char) Serial.read();
    // pagina 32 voor de benodigde opdrachten, vb:
    
    // char, dus enkele aanhalingstekens
    if(input == 'F') {
     //rijnaarvoren(); 
    }
    
    // ....
    
    
    
    
    
  }
}

void loop() {
  // Reset alle info voor een nieuwe run
  int servoAngle = 0;
  s.write(servoAngle);
  int hoogsteBrightness = 0;
  

  while (true) {
    
    // Hebben we nieuwe bluetooth commandos gehad?
    checkBlueTooth();

    int hoogsteAngle = 0;
    hoogsteBrightness = 0;

    // HCSR04 distance sensor afstand
    Serial.println(getDistance());

    // Draai, scan, etc
    for (int i = 0; i <= 18; i++) {
      s.write(10 * i);
      int brightness = getIRBrightness();
      if (brightness > hoogsteBrightness) {
        hoogsteBrightness = brightness;
        hoogsteAngle = 10 * i;
      }
    }
    
    //Print alle IR sensor readouts.
    for(int z = 0; z < 4; z++){
    Serial.println(analogRead(Irbakken[z]));
  }
  
  
    //optimization: we kunnen al exiten voor we de hele loop door zijn als het weer minder wordt:

    if (hoogsteBrightness > threshold) {
      // We hebben nu de hoogste brightness en deze was meer dan de threshold, rijden
      break;
    }
    // Als we hier aankomen dan konden we niks vinden, draaien en nog eens de hele dans doen

  }

  //todo: draai in de goede richting en rijd naar voren
  //rij_voren(500)



  // we komen steeds dichterbij, herhaal de loop tot we er zijn.

}
