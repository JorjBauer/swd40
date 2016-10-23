#include <Arduino.h>
#include <avr/pgmspace.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69
#include <avr/wdt.h>
#include <AH_MCP4921.h>
#include <TimerOne.h>      // used for current speed approximation

#undef DEBUG

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

// FIXME: should we check for the 'changing direction' flags in the main loop, and apply them when we reach 0
// speed in the main loop? Or should we just ignore them, b/c the controller will continue to broadcast, and we 
// will continue to try to set the speed?

/* Pins used.
 *  
 *  The Moteino communicates with its embedded RFM69 on pins 2, 10, 11, 12, 13.
 *  Pin 9 is the onboard LED.
 *  Ping A6 and A7 are analog input only.
 *  
 *   0 (not used, but would be serial in; can't disable b/c using SerialOut)
 *   1 Serial Out (to music board)
 *   2 RFM69: INT0
 *   3 Motor controllers' power key relay
 *   4 SPI CS to DAC for motor #1
 *   5 
 *   6 
 *   7 
 *   8
 *   9 (onboard LED; reusable if necessary)
 *  10 RFM69
 *  11 RFM69 (also: SPI MOSI to DAC)
 *  12 RFM69
 *  13 RFM69 (also: SPI CLK to DAC)
 *  14/A0 
 *  15/A1 
 *  16/A2 gunUpPin
 *  17/A3 gunDownPin
 *  18/A4 shoulderLeftPin
 *  19/A5 shoulderRightPin
 *
 *  If we start running out of pins, we should convert gun and shoulder motors to use a serial protocol.
 */

#define PowerRelayPin 3
#define MotorDACCSPin 4
#define Motor1DirectionPin 5 // low for "reverse"
#define Motor2DirectionPin 7 // low for "reverse"

// Converting to a 4922 - had to modify the library :/
AH_MCP4921 MotorDAC(MotorDACCSPin); // CS pin (uses SPI library)
#define LEFTDAC 0
#define RIGHTDAC 1

#define shoulderLeftPin A4
#define shoulderRightPin A5

#define gunUpPin A2
#define gunDownPin A3

// fixme; these constants belong somewhere else
#define MUSIC_NONE 0

// FLOAT_TIME is how long we should continue to obey a pulse that came in
#define FLOAT_TIME 200
// SLEW_RATE is how fast the motor slows down on its own. We need this b/c the controllers I'm using 
// won't go in to reverse if the motor is still spinning forward (or vice versa, maybe?)
// This is in "units of DAC input per TimerOne interrupt" (an odd metric, to be sure)
#define SLEW_RATE 250

unsigned long MotorTimer = 0;
unsigned long shoulderTimer = 0;
unsigned long brakeTimer = 0;
unsigned long gunTimer = 0;

/* cached settings from last update of remote */
#define kFORWARD 1
#define kBACKWARD -1
int8_t fb_cache = kFORWARD; // forward/backward setting
int8_t p_cache = 0; // percentage motor
bool slowMode = true; // always start in slow mode

/* last set values for the motor speed targets */
int8_t next_left_motor = 0; // current setting, -10 to +10
int8_t next_right_motor = 0; // current setting, -10 to +10
int16_t approximate_left_speed = 0;  // our estimate of how fast the motor is going
int16_t approximate_right_speed = 0; //  ... assumes infinite acceleration and SLEW_RATE decel
int16_t current_left_target = 0;     // How fast we want the motor to be moving
int16_t current_right_target = 0;
bool leftMotorChangingDirection = false;
bool rightMotorChangingDirection = false;

void timerOneInterrupt()
{
  // Update our approximated motor speeds.

  if (approximate_left_speed >= 0 && current_left_target > 0) {
    // accelerating forward: assume infinite acceleration
    approximate_left_speed = current_left_target;
  } else if (approximate_left_speed < 0 && current_left_target < 0) {
    // accelerating backward: assume infinite acceleration
    approximate_left_speed = current_left_target;
  } else if (approximate_left_speed != current_left_target) {
    // decelerating - either forward or backward. Constrain this to our maximum SLEW_RATE.
    int16_t maxDelta = min(SLEW_RATE, abs(approximate_left_speed - current_left_target));
    if (approximate_left_speed > current_left_target) maxDelta = -maxDelta; // get the direction right
    approximate_left_speed += maxDelta;
  }

  if (approximate_right_speed >= 0 && current_right_target > 0) {
    // accelerating forward: assume infinite acceleration
    approximate_right_speed = current_right_target;
  } else if (approximate_right_speed < 0 && current_right_target < 0) {
    // accelerating backward: assume infinite acceleration
    approximate_right_speed = current_right_target;
  } else if (approximate_right_speed != current_right_target) {
    // decelerating - either forward or backward. Constrain this to our maximum SLEW_RATE.
    int16_t maxDelta = min(SLEW_RATE, abs(approximate_right_speed - current_right_target));
    if (approximate_right_speed > current_right_target) maxDelta = -maxDelta; // get the direction right
    approximate_right_speed += maxDelta;
  }
}

void setup()
{
  MCUSR = 0;  // clear out any flags of prior watchdog resets.

  // ensure the CS line is driven very early
  pinMode(MotorDACCSPin, OUTPUT);
  digitalWrite(MotorDACCSPin, HIGH); // active-low, so we want it high...
  delay(1000);


  pinMode(Motor1DirectionPin, OUTPUT);
  digitalWrite(Motor1DirectionPin, HIGH);
  pinMode(Motor2DirectionPin, OUTPUT);
  digitalWrite(Motor2DirectionPin, HIGH);
  
  Serial.begin(9600); // Primarily for talking to the music board
#ifdef DEBUG
  Serial.println("Debug enabled");
#endif

  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
#ifdef DEBUG
    Serial.println("Failed to init RFM69");
    delay(1000);
    ForceRestart();
#endif
  } else {
#ifdef DEBUG
    Serial.println("RFM69 initialized");
#endif  
  }
  radio.encrypt(ENCRYPTKEY);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  
  if (!flash.initialize()) {
#ifdef DEBUG
    Serial.println("Failed to init flash");
#endif    
  } else {
#ifdef DEBUG
    Serial.println("Flash intialized");
#endif    
    
  }

  pinMode(shoulderLeftPin, OUTPUT);
  pinMode(shoulderRightPin, OUTPUT);
  pinMode(gunUpPin, OUTPUT);
  pinMode(gunDownPin, OUTPUT);
  pinMode(PowerRelayPin, OUTPUT);

  pinMode(9, OUTPUT); // LED debugging

  wdt_enable(WDTO_1S);
  
  digitalWrite(PowerRelayPin, LOW); // tell the controllers it's business time! (This is active-low)

  Timer1.initialize(100000); // 0.1 second interrupt period
  Timer1.attachInterrupt(timerOneInterrupt);
}

// Input l/r: [-10 .. +10]
void setMotorTargets(int l, int r)
{
  // The two motor controllers don't perform identically; we'll need to map them individually.
long minValueLeft = 1000;
  long minValueRight = 1000;
  long maxValueLeft = (slowMode ? 1300 : 1400);
  long maxValueRight = (slowMode ? 1300 : 1400);
  int left_abs = map(abs(l), 0, 10, minValueLeft, maxValueLeft); // low-end cutoff is 1v (~800/4096); high point is ~3v (~2500/4096). DAC is 12-bit (0-to-4096)
  int right_abs = map(abs(r), 0, 10, minValueRight, maxValueRight);
  int8_t left_dir = (l >= 0)  ? kFORWARD : kBACKWARD;
  int8_t right_dir = (r >= 0) ? kFORWARD : kBACKWARD;

#ifdef DEBUG
  static char buf[30];
  sprintf(buf, "%d %d => %c%d %c%d\n", l, r, l < 0 ? '-' : '+', left_abs, r <0 ? '-' : '+', right_abs);
  Serial.println(buf);
#endif

  if (left_abs <= minValueLeft) {
    left_abs = 0; // don't set to < 1v; set to 0v instead.
  }
  if (right_abs <= minValueRight) {
    right_abs = 0;
  }

  current_left_target = abs(left_abs) * left_dir;
  current_right_target = abs(right_abs) * right_dir;

  if (current_left_target * approximate_left_speed < 0) {
    // one of them is backward, one is forward; we can't do that. Wait until the motor slows down.
    // FIXME: could apply brake?
    leftMotorChangingDirection = true;
  } else {
    // Apply the change.
    leftMotorChangingDirection = false;
    digitalWrite(Motor1DirectionPin, (left_dir == kFORWARD) ? HIGH : LOW);
    MotorDAC.setValue(left_abs, LEFTDAC);
  }

#ifdef DEBUG
Serial.print("crt: ");
Serial.print(current_right_target);
Serial.print(" ars: ");
Serial.println(approximate_right_speed);
#endif
  if (current_right_target * approximate_right_speed < 0) {
    // one of them is backward, one is forward; we can't do that. Wait until the motor slows down.
    // FIXME: could apply brake?
    rightMotorChangingDirection = true;
  } else {
    // Apply the change.
    rightMotorChangingDirection = false;
    digitalWrite(Motor2DirectionPin, (right_dir == kFORWARD) ? HIGH : LOW);
    MotorDAC.setValue(right_abs, RIGHTDAC);
  }
}

void MakeMotorsGo(int l, int r)
{
  // debugging: flash LED when we get a G pulse
  static bool ledState = 0;
  ledState = !ledState;
  digitalWrite(9, ledState);
  
  setMotorTargets(l, r);
  if (l != 0 || r != 0) {
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
  wdt_reset();
    
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
    if (radio.DATALEN >= 4 && radio.DATA[0] == 'F' && radio.DATA[1] == 'L' && radio.DATA[2] == 'X' && radio.DATA[3] == '?') {
      // probably going to flash - shut down the timer and watchdog timer
      wdt_disable();
    }

    CheckForWirelessHEX(radio, flash, true); // checks for the header 'FLX?' and reflashes new program if it finds one

    bool wantStartMusic = false;
    for (unsigned char i=0; i < radio.DATALEN; i++) {
#ifdef DEBUG2
      // debugging
      static char buf[2] = {0, 0};
      buf[0] = radio.DATA[i];
      Serial.println(buf);
#endif
      
      if (wantStartMusic) {
          startMusic( radio.DATA[i] );
          wantStartMusic = false;
          continue;
      }
  
      switch (radio.DATA[i]) {
        case '*':
          // Restart!
          ForceRestart();
          break;
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

        /* Slow/fast mode */
        case '+':
          slowMode = false;
          break;
        case '-':
          slowMode = true;
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
          setBrake();
          break;
      }
    }
  }
}

void setBrake() {
  ForceRestart(); // FIXME: not what I intended, but good enough for now
}

void ForceRestart()
{
  wdt_enable(WDTO_15MS);
  while(1) ;
}

