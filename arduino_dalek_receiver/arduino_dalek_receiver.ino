#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/pgmspace.h>

#undef DEBUG

/* Pins used.
 *  
 *  Note that some of these are embedded in Jeelib's RF12.cpp. There's no way to 
 *  programmatically check that they're correct here :(
 *  
 *   0 (not used, but would be serial in; can't disable b/c using SerialOut)
 *   1 Serial Out (to music board)
 *   2 
 *   3 M1THROTTLE (PWM)
 *   4 M1DIR
 *   5 M2THROTTLE (PWM)
 *   6 M2DIR
 *   7
 *   8
 *   9 RFM12 RFM_IRQ
 *  10 RFM12 SPI_SS
 *  11 RFM12 SPI_MOSI
 *  12 RFM12 SPI_MISO
 *  13 RFM12 SPI_SS
 *  14/A0 motorKeyPin  
 *  15/A1 motorBrakePin
 *  16/A2 gunUpPin
 *  17/A3 gunDownPin
 *  18/A4 shoulderLeftPin
 *  19/A5 shoulderRightPin
 *
 *  If we start running out of pins, we should convert gun and shoulder motors to use a serial protocol.
 */

#define M1DIR 4
#define M2DIR 6
#define M1THROTTLE 3 // must be a PWM pin
#define M2THROTTLE 5 // must be a PWM pin
#define MotorKeyPin A0
#define MotorBrakePin A1

#define shoulderLeftPin A4
#define shoulderRightPin A5

#define gunUpPin A2
#define gunDownPin A3

#define MaxThrottleVoltage 4.0
#define MOTORMAX 204

// fixme; these constants belong somewhere else
#define MUSIC_NONE 0


// FLOAT_TIME is how long we should continue to obey a pulse that came in
#define FLOAT_TIME 200

// RF channel information
#define RF_NODEID 2
#define RF_GROUPID 212

unsigned long MotorTimer = 0;
unsigned long shoulderTimer = 0;
unsigned long brakeTimer = 0;
unsigned long gunTimer = 0;

/* cached settings from last update of remote */
#define kFORWARD 0
#define kBACKWARD 1
int fb_cache = -1;
int p_cache = -1; // percentage motor

/* last set values for the motor speeds */
int left_motor = 0; // current setting, -10 to +10
int right_motor = 0; // current setting, -10 to +10

void setup()
{
  Serial.begin(9600);
#ifdef DEBUG
  Serial.println("Debug enabled");
#endif

  rf12_initialize(RF_NODEID, RF12_433MHZ, RF_GROUPID);
#ifdef DEBUG
  Serial.println("RF12 initialized");
#endif  
  
  pinMode(shoulderLeftPin, OUTPUT);
  pinMode(shoulderRightPin, OUTPUT);
  pinMode(gunUpPin, OUTPUT);
  pinMode(gunDownPin, OUTPUT);

  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1THROTTLE, OUTPUT);
  pinMode(M2THROTTLE, OUTPUT);
  pinMode(MotorKeyPin, OUTPUT);
  pinMode(MotorBrakePin, OUTPUT);

  setMotorSpeeds(0, 0);
  digitalWrite(MotorKeyPin, HIGH); // assert the key (we're ready to run!)
}

void startMusic(uint8_t which)
{
  Serial.write(which);
}

void SetTimer(unsigned long *t)
{
  *t = millis() + FLOAT_TIME;
}

void setBrake(bool state)
{
  digitalWrite(MotorBrakePin, state);
}

// Given a motor speed in percent (from -100 to 100), enable the appropriate motor settings.
void setMotorSpeeds(int left_motor, int right_motor)
{
  if (left_motor < 0) {
    digitalWrite(M1DIR, HIGH);
  } else {
    digitalWrite(M1DIR, LOW);
  }

  analogWrite(M1THROTTLE, map(abs(left_motor), 0, 100, 0, MOTORMAX));

  if (right_motor < 0) {
    digitalWrite(M2DIR, HIGH);
  } else {
    digitalWrite(M2DIR, LOW);
  }

  analogWrite(M2THROTTLE, map(abs(right_motor), 0, 100, 0, MOTORMAX));
}

void MakeMotorsGo(int left_motor, int right_motor)
{
  // map v from [-10..10] to a percentage from [-100..100]
  int left_out = map(left_motor, -10, 10, -100, 100);
  int right_out = map(right_motor, -10, 10, -100, 100);

  // safety catch for low rouding errors. If we're near zero, assume it should be zero.
  // FIXME: arbitrary constants - this was 5 when MAX_SPEED was 400, so maybe this should be 2 now?
  if (abs(left_out) < 5)
    left_out = 0;
  if (abs(right_out) < 5)
    right_out = 0;
    
  setMotorSpeeds(right_out, left_out);
  if (left_out != 0 || right_out != 0) {
    // If either motor is engaged, then we start the timer
    SetTimer(&MotorTimer);
  }
}

void loop()
{
  /* Spin down the motors if their timers have expired */
  if (MotorTimer && MotorTimer < millis()) {
    MotorTimer = 0;
    setMotorSpeeds(0, 0);
  }

  /* Same with the signaling for the shoulder motors */  
  if (shoulderTimer && shoulderTimer < millis()) {
    shoulderTimer = 0;
    digitalWrite(shoulderLeftPin, LOW);
    digitalWrite(shoulderRightPin, LOW);
  }

  /* Same with the signaling for the gun aiming motor */
  if (gunTimer && gunTimer < millis()) {
    gunTimer = 0;
    digitalWrite(gunUpPin, LOW);
    digitalWrite(gunDownPin, LOW);
  }

  /* Deal with brakes the same way */
  if (brakeTimer && brakeTimer < millis()) {
    brakeTimer = 0;
    setBrake(LOW);
  }
  
  /* See if we have new RF commands waiting to be received */
  
  if (rf12_recvDone()) {

    if (rf12_crc == 0) {
      // CRC==0 means "no errors"

#ifdef DEBUG
      {
        char boo[25];
        sprintf(boo, "rec: [%c]\r\n", rf12_data[0]);
        Serial.print(boo);
      }
#endif
      switch (rf12_data[0]) {
        /* Forward or backward mode for next command */
        case 'F':
          fb_cache = kFORWARD;
          break;
        case 'B':
          fb_cache = kBACKWARD;
          break;
        /* percentage, 0-100% in 10% increments, for next command */
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case ':':
          p_cache = (rf12_data[0] - '0');
          break;
          
        /* Set the left or right motor, when activated, to the current cached state */
        case 'L':
          left_motor = p_cache;
          if (fb_cache == kBACKWARD)
            left_motor = -left_motor;
          break;
        case 'R':
          right_motor = p_cache;
          if (fb_cache == kBACKWARD)
            right_motor = -right_motor;
          break;
          
        /* Pulse the motors at the given values */
        case 'G':
          MakeMotorsGo(left_motor, right_motor);
          break;

        /* Shoulder rotation commands */          
        case '(':
          SetTimer(&shoulderTimer);
          digitalWrite(shoulderRightPin, LOW);
          digitalWrite(shoulderLeftPin, HIGH);
          break;
        case ')':
          SetTimer(&shoulderTimer);
          digitalWrite(shoulderLeftPin, LOW);
          digitalWrite(shoulderRightPin, HIGH);
          break;

        case '^':
          SetTimer(&gunTimer);
          digitalWrite(gunUpPin, HIGH);
          digitalWrite(gunDownPin, LOW);
          break;
        case 'v':
          SetTimer(&gunTimer);
          digitalWrite(gunUpPin, LOW);
          digitalWrite(gunDownPin, HIGH);
          break;
          
        case 'M':
          startMusic( rf12_data[1] );
          break;
        case 'm':
          startMusic( MUSIC_NONE );
          SetTimer(&brakeTimer);
          setBrake(HIGH);
          break;
      }
    }
  }
}


