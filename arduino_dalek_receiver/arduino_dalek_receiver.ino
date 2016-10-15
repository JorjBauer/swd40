#include <Arduino.h>
#include <avr/pgmspace.h>
#include <HoverboardControl.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69
#include <TimerOne.h>

#define DEBUG

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
 *   3 Relay to hoverboard power switch
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

// accel/decel speed
#define ACCEL 15

#define PowerRelayPin 3
#define M1out 4
#define M1in 5
#define M2out 6
#define M2in 7

#define MAXMOTOR 1023 // FIXME: don't know what MAXMOTOR really is. This is based on my limited testing.
#define HEARTBEATTIME 5000 // once every 5 seconds, check power board heartbeat

HoverboardControl leftMotor(M1out, M1in);
HoverboardControl rightMotor(M2out, M2in);

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
unsigned long HeartbeatCheckTimer = 0;

/* cached settings from last update of remote */
#define kFORWARD 0
#define kBACKWARD 1
int fb_cache = kFORWARD; // forward/backward setting
int p_cache = 0; // percentage motor

/* last set values for the motor speed targets */
int next_left_motor = 0; // current setting, -10 to +10
int next_right_motor = 0; // current setting, -10 to +10

int16_t current_left_target = 0;
int16_t current_right_target = 0;
int16_t left_out = 0;
int16_t right_out = 0;

void updateMotors(void)
{
  int8_t la = 0, ra = 0; // accelerations - replicating what I think is the IMU acceleration data?

  // ramp up and down to our targets.
  if (left_out != current_left_target) {
    if (current_left_target > left_out) {
      la = 1;
    } else {
      // must be less than
      la = -1;
    }
    left_out += (la * ACCEL);
    // check for overshoot b/c ACCEL. :/
    if (la > 0 && left_out > current_left_target)
      left_out = current_left_target;
    if (la < 0 && left_out < current_left_target)
      left_out = current_left_target;
  }

  if (right_out != current_right_target) {
    if (current_right_target > right_out) {
      ra = 1;
    } else {
      // must be less than
      ra = -1;
    }
    right_out += (ra * ACCEL);
    // check for overshoot b/c ACCEL. :/
    if (ra > 0 && right_out > current_right_target)
      right_out = current_right_target;
    if (ra < 0 && right_out < current_right_target)
      right_out = current_right_target;
  }

  // convert left_out and right_out in to an angle. The default near-level angle is 
//0x31 0x00 0x31 0x00 0x55 0x2B 0x2B 0x00 0x00 0x80 


  leftMotor.write9(left_out & 0xFF);
  leftMotor.write9((left_out >> 8) & 0xFF);
  leftMotor.write9(left_out & 0xFF);
  leftMotor.write9((left_out >> 8) & 0xFF);
  leftMotor.write9(0x55); // magic number. ... 0xAA is the other one it uses, but that doesn't do diddley.
  leftMotor.write9(0x2C); // Don't know what this is, but 
  leftMotor.write9(0x2C); //   ... it has to be repeated. Values vary from ~70 to ~100 in "normal" operation, from what I see.
  leftMotor.write9(0);  // Don't know what this is either, but
  leftMotor.write9(0);  //   ... it has to be repeated too. Might be accel? seems to vary from a small negative to a small positive.
  leftMotor.write9(0x100);

  rightMotor.write9(right_out & 0xFF);
  rightMotor.write9((right_out >> 8) & 0xFF);
  rightMotor.write9(right_out & 0xFF);
  rightMotor.write9((right_out >> 8) & 0xFF);
  rightMotor.write9(0x55); // magic number. ... 0xAA is the other one it uses, but that doesn't do diddley.
  rightMotor.write9(80); // Don't know what this is, but 
  rightMotor.write9(80); //   ... it has to be repeated. Values vary from ~70 to ~100 in "normal" operation, from what I see.
  rightMotor.write9(0);  // Don't know what this is either, but
  rightMotor.write9(0);  //   ... it has to be repeated too. Might be accel? seems to vary from a small negative to a small positive. Seems to make beeps.
  rightMotor.write9(0x100);
}

void PulseMainBoardPowerRelay()
{
  // Force motor state back to zero; we're apparently not in communication with the driver board yet
  left_out = right_out = current_left_target = current_right_target = 0;
  
  digitalWrite(PowerRelayPin, HIGH);
  delay(500);
  digitalWrite(PowerRelayPin, LOW);
}

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
  pinMode(PowerRelayPin, OUTPUT);

  pinMode(9, OUTPUT); // LED debugging

  Timer1.initialize(10000);
  Timer1.attachInterrupt(updateMotors); // attaches the interrupt
  Timer1.start(); // starts the timer

  HeartbeatCheckTimer = millis() + HEARTBEATTIME;
}

void setMotorTargets(int l, int r)
{
  current_left_target = l;
  current_right_target = r;
}

void MakeMotorsGo(int l, int r)
{
  // debugging: flash LED when we get a G pulse
  static bool ledState = 0;
  ledState = !ledState;
  digitalWrite(9, ledState);
  
  // map v from [-10..10] to a percentage from [-100..100]
  int lo = map(l, -10, 10, -MAXMOTOR, MAXMOTOR);
  int ro = map(r, -10, 10, -MAXMOTOR, MAXMOTOR);

  // debugging
  static char buf[30];
  sprintf(buf, "%d %d => %d %d\n", l, r, lo, ro);
  Serial.println(buf);

  // safety catch for low rouding errors. If we're near zero, assume it should be zero.
  // FIXME: arbitrary constants - this was 5 when MAX_SPEED was 400, so maybe this should be 2 now?
  if (abs(lo) < 5)
    lo = 0;
  if (abs(ro) < 5)
    ro = 0;

  setMotorTargets(lo, ro);
  if (lo != 0 || ro != 0) {
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

void loop()
{
  /* Periodically check for communication from the main drive board. It will shut itself down if it's left inactive... */
  if (HeartbeatCheckTimer && HeartbeatCheckTimer < millis()) {
    HeartbeatCheckTimer = millis() + HEARTBEATTIME;
    if (!leftMotor.isAlive()) {
      PulseMainBoardPowerRelay();
    }
}
  
  /* Spin down the motors if their timers have expired */
  if (MotorTimer && MotorTimer < millis()) {
    MotorTimer = 0;
    setMotorTargets(0, 0);
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
#if 0
  if (brakeTimer && brakeTimer < millis()) {
    brakeTimer = 0;
//    setBrake(LOW);
  }
#endif
  
  /* See if we have new RF commands waiting to be received */
  if (radio.receiveDone()) {
    CheckForWirelessHEX(radio, flash, true); // checks for the header 'FLX?' and reflashes new program if it finds one

    bool wantStartMusic = false;
    for (unsigned char i=0; i < radio.DATALEN; i++) {
      // debugging
      static char buf[2] = {0, 0};
      buf[0] = radio.DATA[i];
      Serial.println(buf);
      
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
          next_left_motor = p_cache;
          if (fb_cache == kBACKWARD)
            next_left_motor = -next_left_motor;
          break;
        case 'R':
          next_right_motor = p_cache;
          if (fb_cache == kBACKWARD)
            next_right_motor = -next_right_motor;
          break;
          
        /* Pulse the motors at the given values */
        case 'G':
          MakeMotorsGo(next_left_motor, next_right_motor);
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

