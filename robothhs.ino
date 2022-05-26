#include <Servo.h>

//todo trigger pin echo pin distout distin
// linksachter motor2
// linksvoor M1
// rechtsvoor m4
// rechtsachter m3

bool manual = 0;

// TODO CHANGE THESE PINS
#define sensorLinks 15
#define sensorRechts 19

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
#define FORWARD 0
#define BACKWARD 2
#define RELEASE 4

#define LEFT 1
#define RIGHT 3
#define TurnLeft 5
#define TurnRight 6

#define servoAnglelinks 45
#define servoAngleRechtdoor 90
#define servoAngleRechts 135

// Motor frequency definitions (all 1khz, may be audible but best torque)
#define MOTOR12_1KHZ _BV(CS22)              // divide by 64
#define MOTOR34_1KHZ _BV(CS01) | _BV(CS00)  // divide by 64

#define MOTOR12_8KHZ _BV(CS21)              // divide by 8
#define MOTOR34_8KHZ _BV(CS01)              // divide by 8

// Servo
#define servoPin 10
Servo s;

//Afstand van 1 blok
#define afstandblok 900

// Stored the state of the shift register
static uint8_t latch_state;

// Poort indices voor de IR baken sensoren.
int IRBakens[] = {A2, A3, A4, A5};
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
void setSpeed(uint8_t speed) {
  // OCR = output compare register
  // Je timer telt op tot 255, en gaat uit als de waarde van het OCR* register gehaald is.
  // Hierdoor krijg je een duty cycle op een schaal van 0-255

  // motor 1, use PWM from timer2A on PB3 (Arduino pin #11)
  //  OCR2A = speed;

  // motor 2, use PWM from timer2B (pin 3)
  // OCR2B = speed;

  // motor 3, use PWM from timer0A / PD6 (pin 6)
  // OCR0A = speed;

  // motor 4, use PWM from timer0B / PD5 (pin 5)
  // OCR0B = speed;

  analogWrite(11, speed);
  analogWrite(3, speed);
  analogWrite(6, speed);
  analogWrite(5, speed);
}


// zet alle motoren op 0 en stel de timers in.
void initMotor(uint8_t num, uint8_t freq) {

  switch (num) {
    case 1:
      latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
      latch_tx();
      //initPWM1(freq);
      break;
    case 2:
      latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
      latch_tx();
      //initPWM2(freq);
      break;
    case 3:
      latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
      latch_tx();
      // initPWM3(freq);
      break;
    case 4:
      latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
      latch_tx();
      // initPWM4(freq);
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
    drive(BACKWARD, LA);
    drive(FORWARD, RA);
    drive(FORWARD, LV);
    drive(BACKWARD, RV);
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
  initMotor(1, MOTOR12_8KHZ);
  initMotor(2, MOTOR12_8KHZ);
  initMotor(3, MOTOR34_8KHZ);
  initMotor(4, MOTOR34_8KHZ);

  // speed (duty cycle) up to 255, maar tot 180 veilig
  setSpeed(180);

  //Possible options: FORWARD, BACKWARD, RELEASE, LEFT, RIGHT, TurnLeft, TurnRight
  driveDirection(TurnRight);
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
    switch (input) {
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

//void voorlinksrechts() {
//
//
//  driveDirection(FORWARD);
//  //Obstakel staat voor het wagen
//  int VV = getDistance(); //de opgeslagen afstand van voor
//  if (VV < 20 || VV > 900) {
//    driveDirection(RELEASE);
//    //draait servo dan naar links en checkt of er links een muur zit
//    s.write(servoAnglelinks);
//    delay(100);
//    int LL = getDistance(); //de opgeslagen afstand van links
//    if ((LL < 20 || LL > 900)) {
//      delay(100); // luc
//      //links zit een muur dan gaan we naar rechts kijken met UR sensor
//      s.write(servoAngleRechts);
//      int RR = getDistance(); //de opgeslagen afstand van rechts
//      if (RR > 10  && RR < 900 ) {
//        //als rechts ruimte is dan gaan we rechts
//        driveDirection(TurnRight);
//        delay(150);
//        driveDirection(RELEASE);
//        delay(100);
//        s.write(servoAngleRechtdoor);
//        driveDirection(FORWARD);
//        //Als links en rechts een muur zit dan rijden we even naar achter
//      } else {
//        driveDirection(BACKWARD);
//        delay(300);
//      }
//      //als linker kant vrij is dan gaat het naar links
//    } else {
//      driveDirection(TurnLeft);
//      delay(150);
//      driveDirection(RELEASE);
//      delay(100);
//      s.write(servoAngleRechtdoor);
//      driveDirection(FORWARD);
//    }
//  }
//
//}

struct IRWaarden {
  int maxValue;
  int maxDirection;
  int IRWaarden[5];
};

struct IRWaarden getIRDirection() {
  int IRWaardenArray[5] = { 0 };

  int maxVal = 0;
  int maxZ = 0;
  int secondMaxDirection = 0;
  int secondMaxValue = 0;

  for (int z = 0; z < 4; z++) {
    //if(z==1){continue;}
    // doe een lezing en sla deze op
    IRWaardenArray[z] = analogRead(IRBakens[z]);

    // Is dit de hoogste waarde?
    if (IRWaardenArray[z] > maxVal) {

      secondMaxValue = maxVal;
      secondMaxDirection = maxZ;
      maxVal = IRWaardenArray[z];
      maxZ = z;
    }


  }

  struct IRWaarden waarde = { maxVal, maxZ };
  // Workaround omdat c++ geen arrays in structs wil hebben zonder pointers.
  memcpy(waarde.IRWaarden, IRWaardenArray, sizeof(int) * 5);
  return waarde;
}


struct IRWaarden IRcalibreren() {
  int pogingen = 0;
  struct IRWaarden waarden;
  while (true) {

    waarden = getIRDirection();

    // Draai tot we een waarde boven de 50 hebben of als we het 20 keer geprobeert hebben.
    if (waarden.maxValue > 50 || pogingen > 20) {
      break;
    } else {
      driveDirection(TurnLeft);
      delay(25);
      driveDirection(RELEASE);
      pogingen++;

    }

  }
  return waarden;

}

void richtingCalibreren() {

  bool muurlinks = digitalRead(sensorLinks);
  bool muurrechts = digitalRead(sensorRechts);

  if (muurlinks && !muurrechts) {
    // kort naar achter en dan rechts
    driveDirection(BACKWARD);
    delay(10);
    driveDirection(RIGHT);
    delay(30);
    driveDirection(RELEASE);
  } else if (!muurlinks && muurrechts) {
    // kort naar achter en dan links.
    driveDirection(BACKWARD);
    delay(10);
    driveDirection(LEFT);
    delay(30);
    driveDirection(RELEASE);
  }



}

void zigzag() {
  // links
  driveDirection(TurnLeft);
  delay(25);
  driveDirection(FORWARD);
  delay(40);
  struct IRWaarden links = getIRDirection();
  // midden
  driveDirection(TurnRight);
  delay(25);
  driveDirection(FORWARD);
  delay(40);
  struct IRWaarden midden = getIRDirection();
  // rechts
  driveDirection(TurnRight);
  delay(25);
  driveDirection(FORWARD);
  delay(40);
  struct IRWaarden rechts = getIRDirection();
  Serial.print("links maxval: ");
  Serial.println(links.maxValue);
  if (links.maxValue > rechts.maxValue && links.maxValue > midden.maxValue) {
    // links zagen we het meeste
    driveDirection(TurnLeft);
    delay(45); // iets korter omdat we in 1x door kunnen draaien

    // TODO hier een while loop van maken zodat we stoppen als het karretje voorbij is of we een muur raken
    if (links.maxValue > 50) {
      driveDirection(links.maxDirection);
      int waarde = analogRead(links.maxDirection);
      while (waarde > 30) {
        waarde = analogRead(links.maxDirection);
        delay(20);
      }
    }

  } else  if (midden.maxValue > links.maxValue && midden.maxValue > rechts.maxValue) {
    // midden zagen we het meeste
    driveDirection(TurnLeft);
    delay(25);


    // TODO hier een while loop van maken zodat we stoppen als het karretje voorbij is of we een muur raken
    if (midden.maxValue > 50) {
      driveDirection(midden.maxDirection);
      int waarde = analogRead(midden.maxDirection);
      while (waarde > 30) {
        waarde = analogRead(midden.maxDirection);
        delay(20);
      }
    }

  } else {
    // We staan al rechts, dus we hoeven niet te draaien.

    // TODO hier een while loop van maken zodat we stoppen als het karretje voorbij is of we een muur raken
    if (rechts.maxValue > 50) {
      driveDirection(rechts.maxDirection);
      int waarde = analogRead(rechts.maxDirection);
      while (waarde > 30) {
        waarde = analogRead(rechts.maxDirection);
        delay(10);
      }
    }

  }

  // Iets naar voren presumably
  if (getDistance() > 15) {
    driveDirection(FORWARD);
  }




}

// The main loop for our program.
// Delays are kept as low as possible to have a higher chance of the IR beacon picking up a vehicle driving past.
void loop() {


  // Hebben we nieuwe bluetooth commandos gehad?
  checkBlueTooth();
  if (manual) {
    return; // If manual, don't run through the algorithm.
  }


  // Nothing in front of us
  if (getDistance() > 20) {
    // zigzag();
    struct IRWaarden meting = getIRDirection();
    Serial.print("max val: ");
    Serial.println(meting.maxValue);
    // IR beacon in one of the 4 sides
    if (meting.maxValue > 200) {
        // Direct op af

      driveDirection(meting.maxDirection);
      delay(300);

    }else if(meting.maxValue > 20){
      int tries = 0;
      // Turn until max meting, should probably filter outliers
        if(meting.maxDirection == LEFT){
          driveDirection(TurnLeft)
        }else if(meting.maxDirection == RIGHT || meting.maxDirection == BACKWARD){
          driveDirection(TurnRight)
        }
        // Turn in 10ms amounts until we have something equal or better than the last measurement in front of us.
        // Give up after 500ms
        while(analogRead(FORWARD) <= meting.maxValue && tries < 50){
          tries++;
          delay(10);
        }
        driveDirection(FORWARD);
        if(tries <= 49){
          delay(100);
        }

        // Zigzag to it
        //zigazg();

    }

      // driveDirection(meting.maxDirection);
      // // TODO actually follow w/ zigzag.
      // // We might hit walls here driving sidewards/back but probably won't get stuck as the goal is moving.

      // // Drive a bit towards our goal, then start zigzagging.
      // delay(100);

      // int tries = 0;
      // while(tries < 25){
      //   zigzag();
      //   delay(40);
      //   tries++;
      // }

      // // Rerun the script if we haven't found it within 25 tries. It probably escaped
      

    } else {
      // Just drive around aimlessly until we get a reading
      if (digitalRead(sensorRechts) == 0 && digitalRead(sensorLinks) == 1) {
          // Muur rechts
          driveDirection(LEFT);
          delay(15);
      }
      if (digitalRead(sensorRechts) == 1 && digitalRead(sensorLinks) == 0) {
        // Muur links
          driveDirection(RIGHT);
          delay(15);
      }
      driveDirection(FORWARD);
      delay(30);
    }

  // Something in front of us, Try to get past.
  // Prefers turning right
  } else {
    // TODO add dist check links voor we dit doen
    driveDirection(BACKWARD);
    delay(10);
    // If there is a wall on our right, turn left instead.
    if (digitalRead(sensorRechts) == 0) {

      driveDirection(LEFT);
      delay(5);
      driveDirection(TurnLeft);
      delay(10);

    } else {

      driveDirection(RIGHT);
      delay(5);
      driveDirection(TurnRight);
      delay(10);
    }
  }


}
