#include <Servo.h>

//todo trigger pin echo pin distout distin
// linksachter motor2
// linksvoor M1
// rechtsvoor m4
// rechtsachter m3

bool manual = 0;

// TODO CHANGE THESE PINS
#define sensorLinks 13
#define sensorRechts 9

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


// Set duty cycles
void setSpeed(uint8_t speed) {

  analogWrite(11, speed);
  analogWrite(3, speed);
  analogWrite(6, speed);
  analogWrite(5, speed);
}


// zet alle motoren op 0.
void initMotor(uint8_t num) {

  switch (num) {
    case 1:
      latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
      latch_tx();
      break;
    case 2:
      latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
      latch_tx();
      break;
    case 3:
      latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
      latch_tx();
      break;
    case 4:
      latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
      latch_tx();
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

  pinMode(sensorLinks, INPUT);
  pinMode(sensorRechts, INPUT);

  latch_state = 0; // reset all motor states
  latch_tx();

  // Enable motor
  digitalWrite(MOTORENABLE, LOW);

  initMotor(1);
  initMotor(2);
  initMotor(3);
  initMotor(4);

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


struct IRWaarden {
  int maxValue;
  int maxDirection;
};

struct IRWaarden getIRDirection() {
  int IRWaardenArray[5] = { 0 };

  int maxValue = 0;
  int hoogsteRichting = 0;
  int secondMaxDirection = 0;
  int secondmaxValue = 0;

  for (int richting = 0; richting < 4; richting++) {
    // doe een lezing en sla deze op
    IRWaardenArray[richting] = analogRead(IRBakens[richting]);

    // Is dit de hoogste waarde?
    if (IRWaardenArray[richting] > maxValue) {

      secondmaxValue = maxValue;
      secondMaxDirection = hoogsteRichting;
      maxValue = IRWaardenArray[richting];
      hoogsteRichting = richting;
    }

  }

  struct IRWaarden waarde = { maxValue, hoogsteRichting };
  return waarde;
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
    struct IRWaarden waarden = getIRDirection();
    // IR beacon in one of the 4 sides
    if (waarden.maxValue > 20) {
      driveDirection(waarden.maxDirection);
      delay(300);
    } else {
      // Just drive around aimlessly until we get a reading

      // Lees de sensoren aan de zijkant uit, 0 is hier een muur.
      if (digitalRead(sensorRechts) == 0 && digitalRead(sensorLinks) == 1) {
        // Muur rechts
        driveDirection(LEFT);
        delay(70);
      }
      if (digitalRead(sensorRechts) == 1 && digitalRead(sensorLinks) == 0) {
        // Muur links
        driveDirection(RIGHT);
        delay(70);
      }
      driveDirection(FORWARD);
      delay(30);
    }

    // Something in front of us, Try to get past.
  } else {
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
