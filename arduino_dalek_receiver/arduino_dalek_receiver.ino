#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/pgmspace.h>
#include "DualMC33926MotorShield.h"

#undef DEBUG

#define M1DIR 7
#define M2DIR 8
#define M1PWM 5 // originally pin 9; re-soldered the circuit
#define M1FB A0
#define M2DIR 8
#define M2PWM 6 // originally pin 10; re-soldered the circuit
#define M2FB A1
#define D2 4
#define SF 2 // originally pin 12; re-soldered the circuit
DualMC33926MotorShield md(M1DIR, M1PWM, M1FB, M2DIR, M2PWM, M2FB, D2, SF);

#define shoulderLeftPin A4
#define shoulderRightPin A5

// fixme; these constants belong somewhere else
#define MUSIC_NONE 0
#define MUSIC_INTER1 '1'
#define MUSIC_INTER2 '2'
#define MUSIC_INTER3 '3'
#define MUSIC_INTER4 '4'
#define MUSIC_GUN '5'
#define MUSIC_EXTERM '6'
#define MUSIC_THEME '7'


// FLOAT_TIME is how long we should continue to obey a pulse that came in
#define FLOAT_TIME 200

// RF channel information
#define RF_NODEID 2
#define RF_GROUPID 212

// Speeds are a max of 400, min of -400
#define MAX_SPEED 400
// Divide any backward-motion speeds by 2, because the Dalek is top-heavy on the back
#define REVERSE_DIVISOR 2

unsigned long MotorTimer = 0;
unsigned long shoulderTimer = 0;

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
  md.init();
  rf12_initialize(RF_NODEID, RF12_433MHZ, RF_GROUPID);
  
  pinMode(shoulderLeftPin, OUTPUT);
  pinMode(shoulderRightPin, OUTPUT);
}

void startMusic(uint8_t which)
{
  Serial.write(which);
}

void SetTimer(unsigned long *t)
{
  *t = millis() + FLOAT_TIME;
}

void MakeMotorsGo(int left_motor, int right_motor)
{
  // map v from [-10..10] to [-MAX_SPEED..MAX_SPEED]
  // and then we need to pfutz with it a little.
  int left_out = map(left_motor, -10, 10, -MAX_SPEED, MAX_SPEED);
  int right_out = map(right_motor, -10, 10, -MAX_SPEED, MAX_SPEED);

  // safety catch for low rouding errors...
  if (abs(left_out) < 5)
    left_out = 0;
  if (abs(right_out) < 5)
    right_out = 0;
    
  // if it's a no-op, then shut down the motors and we're done.
  if (left_out == 0 && right_out == 0) {
    md.setSpeeds(0, 0);
    return;
  }
  
#if 0
  if (left_out || right_out ) {
    // If it's not purely rotation, then we need to also take in to account 
    // that the Dalek is top-heavy. Don't go backward too fast.
    
    if (left_out < 0) {
      left_out /= REVERSE_DIVISOR;
    }
    if (right_out < 0) {
      right_out /= REVERSE_DIVISOR;
    }
  }
#endif

  md.setSpeeds(right_out, left_out);
  
  SetTimer(&MotorTimer);
}

void loop()
{
  /* Spin down the motors if their timers have expired */
  if (MotorTimer && MotorTimer < millis()) {
    MotorTimer = 0;
    md.setM1Speed(0);
    md.setM2Speed(0);
  }

  /* Same with the signaling for the shoulder motors */  
  if (shoulderTimer && shoulderTimer < millis()) {
    shoulderTimer = 0;
    digitalWrite(shoulderLeftPin, LOW);
    digitalWrite(shoulderRightPin, LOW);
  }
  
  /* See if we have new RF commands waiting to be received */
  
  if (rf12_recvDone()) {
    if (rf12_crc == 0) {
      // CRC==0 means "no errors"

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
          
        case 'M':
          startMusic( rf12_data[1] );
          break;
        case 'm':
          startMusic( MUSIC_NONE );
          break;
      }
    }
  }
}


