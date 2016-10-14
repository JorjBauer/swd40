#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial9.h> // get it here: https://github.com/addibble/SoftwareSerial9
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69

#undef DEBUG

/*
 * Possible motor/power initialization protocol
 * send 0 0 0 0 170 0 0 0 0 256 at least once
 *   expect the return line to be high
 *   if it's not high, then we press the power button; wait some time (~40-50mS?); try again
 *   
 * In normal operation, it looks like the return line goes low for ~0.19mS regularly, 
 *   occasionally (37.875 uS off; 19 uS on;0.1136875mS on). So, in theory, if the line 
 *   isn't high - we can wait 0.19mS and poll again; if it's still not high, we have to 
 *   momentarily "press" the power button.
 * (I think the fluctuations could be the controller falling just under the logic level 
 *   limit while in mid-swing? Not sure the data is meaningful.)
 * So we could say that, at any time, we could check to see if the motors are responding;
 *   and if not, then we power up the control board.
 * Should think about how this interacts with the Dalek sitting idle for a long time; 
 *   board shuts down; then Arduino gets a command to move, and has to figure out that 
 *   the main board needs to be turned on.
 */

/* Moteino constants */
#define NODEID      30
#define NETWORKID   212
#define FREQUENCY RF69_915MHZ
//#define IS_RFM69HW  //uncomment only for RFM69HW! Leave out if you have RFM69W!
#ifdef DEFAULTKEY
#define ENCRYPTKEY DEFAULTKEY
#else
#pragma message("Default encryption key not found; using compiled-in default instead")
#define ENCRYPTKEY "sampleEncryptKey"
#endif
#define FLASH_SS 8

RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30); // 0xEF30 is windbond 4mbit

// Speed of the controller's bus (in baud). Note that the Cortex M3 on the controller has CAN bus support, and the 
// CAN bus supports 26315.79 baud as a "normal" speed (but still unusual to see it).
#define BUSSPEED 53156

/* Pins used.
 *  
 *  The Moteino communicates with its embedded RFM69 on pins 2, 10, 11, 12, 13.
 *  Pin 9 is the onboard LED.
 *  Ping A6 and A7 are analog input only.
 *  
 *   0 (not used, but would be serial in; can't disable b/c using SerialOut)
 *   1 Serial Out (to music board)
 *   2 RFM69: INT0
 *   3 
 *   4 M1 out
 *   5 M1 in
 *   6 M2 out
 *   7 M2 in
 *   8
 *   9 (onboard LED; reusable if necessary)
 *  10 RFM69
 *  11 RFM69
 *  12 RFM69
 *  13 RFM69
 *  14/A0 
 *  15/A1 
 *  16/A2 gunUpPin
 *  17/A3 gunDownPin
 *  18/A4 shoulderLeftPin
 *  19/A5 shoulderRightPin
 *
 *  If we start running out of pins, we should convert gun and shoulder motors to use a serial protocol.
 */

#define M1out 4
#define M1in 5
#define M2out 6
#define M2in 7

#define MAXMOTOR 2048 // FIXME: don't know what MAXMOTOR really is. This is based on my limited testing. I think it goes as high as 

SoftwareSerial9 leftMotor(M1in, M1out);
SoftwareSerial9 rightMotor(M2in, M2out);

#define shoulderLeftPin A4
#define shoulderRightPin A5

#define gunUpPin A2
#define gunDownPin A3

// fixme; these constants belong somewhere else
#define MUSIC_NONE 0

// FLOAT_TIME is how long we should continue to obey a pulse that came in
#define FLOAT_TIME 200

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
  Serial.begin(9600); // Primarily for talking to the music board
#ifdef DEBUG
  Serial.println("Debug enabled");
#endif

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.encrypt(ENCRYPTKEY);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
#ifdef DEBUG
  Serial.println("RFM69 initialized");
#endif  
  flash.initialize();

  // FIXME: the startup timing here is fairly important. We have to be transmitting when the main controller board comes online or it will ignore us.
  leftMotor.begin(BUSSPEED);
  rightMotor.begin(BUSSPEED);

  pinMode(shoulderLeftPin, OUTPUT);
  pinMode(shoulderRightPin, OUTPUT);
  pinMode(gunUpPin, OUTPUT);
  pinMode(gunDownPin, OUTPUT);

//  initMotors();

#if 0
/* DEBUG START */
  while (1) {
    uint16_t a, b, c, d, e;
    short program_counter = 0;
    do {
      a = pgm_read_byte((const uint8_t *)&program[program_counter++]);
      b = pgm_read_byte((const uint8_t *)&program[program_counter++]);
      c = pgm_read_byte((const uint8_t *)&program[program_counter++]);
      d = pgm_read_byte((const uint8_t *)&program[program_counter++]);
      e = pgm_read_byte((const uint8_t *)&program[program_counter++]);
  
      if (a || b || c || d || e) {
        leftMotor.write9(a);
        leftMotor.write9(b);
        leftMotor.write9(a);
        leftMotor.write9(b);
        leftMotor.write9(c);
        leftMotor.write9(d);
        leftMotor.write9(d);
        leftMotor.write9(e);
        leftMotor.write9(e);
        leftMotor.write9(0x100);
      }
    } while (a || b || c || d || e);
  }
/* DEBUG END */
#endif
}

void initMotors()
{
  byte initVectors[] = { 0,0,170,0,0,
0,0,170,0,0,
0,0,170,0,0,
0,0,170,0,0,
0,0,170,0,0,
0,0,170,0,0,
0,0,170,0,0,
0,0,170,0,0,
0, 0, 0, 0, 0 };

  Serial.println("init");

  int ptr = 0;
  while (1) {
    byte a = initVectors[ptr++];
    byte b = initVectors[ptr++];
    byte c = initVectors[ptr++];
    byte d = initVectors[ptr++];
    byte e = initVectors[ptr++];
    if (a || b || c || d || e) {
        leftMotor.write9(a);
        leftMotor.write9(b);
        leftMotor.write9(a);
        leftMotor.write9(b);
        leftMotor.write9(c);
        leftMotor.write9(d);
        leftMotor.write9(d);
        leftMotor.write9(e);
        leftMotor.write9(e);
        leftMotor.write9(0x100);
    } else {
      break;
    }
  }
}

void setMotorSpeeds(int left_out, int right_out, signed char direction)
{
      leftMotor.write9(left_out & 0xFF);
      leftMotor.write9((left_out >> 8) & 0xFF);
      leftMotor.write9(left_out & 0xFF);
      leftMotor.write9((left_out >> 8) & 0xFF);
      leftMotor.write9(0x55); // magic number. ... 0xAA is the other one it uses, but that doesn't do diddley.
      leftMotor.write9(80); // Don't know what this is, but 
      leftMotor.write9(80); //   ... it has to be repeated. Values vary from ~70 to ~100 in "normal" operation, from what I see.
      leftMotor.write9(0);  // Don't know what this is either, but
      leftMotor.write9(0);  //   ... it has to be repeated too. Might be angle? seems to vary from a small negative to a small positive.
      leftMotor.write9(0x100);

#if 1
      rightMotor.write9(right_out & 0xFF);
      rightMotor.write9((right_out >> 8) & 0xFF);
      rightMotor.write9(right_out & 0xFF);
      rightMotor.write9((right_out >> 8) & 0xFF);
      rightMotor.write9(0x55); // magic number. ... 0xAA is the other one it uses, but that doesn't do diddley.
      rightMotor.write9(80); // Don't know what this is, but 
      rightMotor.write9(80); //   ... it has to be repeated. Values vary from ~70 to ~100 in "normal" operation, from what I see.
      rightMotor.write9(0);  // Don't know what this is either, but
      rightMotor.write9(0);  //   ... it has to be repeated too. Might be angle? seems to vary from a small negative to a small positive.
      rightMotor.write9(0x100);
#endif

//   delay(100);
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
    
  setMotorSpeeds(right_out, left_out, 0);
  if (left_out != 0 || right_out != 0) {
    // If either motor is engaged, then we start the timer
    SetTimer(&MotorTimer);
  }
}

void startMusic(uint8_t which)
{
  Serial.write(which);
}

void SetTimer(unsigned long *t)
{
  *t = millis() + FLOAT_TIME;
}

// loop for debugging: try ramping up and down continually
void loop()
{
  signed int speed = 0;
  signed char direction = -1;
  while (1) {
    setMotorSpeeds(speed, speed, direction);
    speed += direction;
    if (abs(speed) == MAXMOTOR) {
      direction = -direction;
    }
  }
}

void loop2()
{
  /* Spin down the motors if their timers have expired */
  if (MotorTimer && MotorTimer < millis()) {
    MotorTimer = 0;
    setMotorSpeeds(0, 0, 0);
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
//    setBrake(LOW);
  }
  
  /* See if we have new RF commands waiting to be received */
  if (radio.receiveDone()) {
    CheckForWirelessHEX(radio, flash, true); // checks for the header 'FLX?' and reflashes new program if it finds one

    bool wantStartMusic = false;
    for (unsigned char i=0; i < radio.DATALEN; i++) {
      if (wantStartMusic) {
          startMusic( radio.DATA[i] );
          wantStartMusic = false;
          continue;
      }
  
      switch (radio.DATA[i]) {
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
          p_cache = (radio.DATA[i] - '0');
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
          wantStartMusic=true;
          break;
        case 'm':
          startMusic( MUSIC_NONE );
          SetTimer(&brakeTimer);
//          setBrake(HIGH);
          break;
      }
    }
  }
}

