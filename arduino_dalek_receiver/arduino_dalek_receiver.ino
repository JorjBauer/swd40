#include <Arduino.h>
#include <avr/pgmspace.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69
#include <avr/wdt.h>
#include <AH_MCP4921.h>
#include <TimerOne.h>      // used for current speed approximation
#include <SoftwareSerial.h>

#undef DEBUG // note: interferes with music board operation, b/c that's also on the serial line. Only useful for debugging when the music controller is disconnected.

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

#define SSERRX 9 // Not used; doubled up with onboard LED
#define SSERTX A0

RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30); // 0xEF30 is windbond 4mbit
SoftwareSerial softSerial(SSERRX, SSERTX);
/* Pins used.
 *  
 *  The Moteino communicates with its embedded RFM69 on pins 2, 10, 11, 12, 13.
 *  Pin 9 is the onboard LED.
 *  Ping A6 and A7 are analog input only.
 *  
 *   0 (not used, but would be serial in; can't disable b/c using SerialOut)
 *   1 Serial Out to music board; 9600 baud
 *   2 RFM69: INT0
 *   3 Right brake pin (drag to ground for enabled)
 *   4 SPI CS to (dual) DAC for both motors
 *   5 Left direction pin (drag to ground for reverse)
 *   6 Left brake engaged (drag to ground for enabled)
 *   7 Right direction pin (drag to ground for reverse)
 *   8 Flash /SS pin
 *   9 (onboard LED; reusable if necessary)
 *  10 RFM69
 *  11 RFM69 (also: SPI MOSI to DAC)
 *  12 RFM69
 *  13 RFM69 (also: SPI CLK to DAC)
 *  14/A0 Software Serial out to shoulder controller board (9600 baud)
 *  15/A1 
 *  16/A2 
 *  17/A3 
 *  18/A4 DAC SCK
 *  19/A5 DAC SDI
 *
 */

#define MotorDACCSPin 4
#define Motor1DirectionPin 5 // low for "reverse"
#define Motor2DirectionPin 7 // low for "reverse"
#define Motor1BrakePin 6 // low for "brake engaged"
#define Motor2BrakePin 3 // low for "brake engaged"

#define DAC_SDI_PIN A5
#define DAC_SCK_PIN A4

// Converting to a 4922 - had to modify the library :/
//AH_MCP4921 MotorDAC(MotorDACCSPin); // CS pin (uses SPI library)
AH_MCP4921 MotorDAC(DAC_SDI_PIN, DAC_SCK_PIN, MotorDACCSPin); // Using bitbang technique
#define LEFTDAC 0
#define RIGHTDAC 1

// fixme; these constants belong somewhere else
#define MUSIC_NONE 0

// FLOAT_TIME is how long we should continue to obey a pulse that came in
#define FLOAT_TIME 200
// SLEW_RATE is how fast the motor slows down on its own. We need this b/c the controllers I'm using 
// won't go in to reverse if the motor is still spinning forward (or vice versa, maybe?)
// This is in "units of DAC input per TimerOne interrupt" (an odd metric, to be sure)
#define SLEW_RATE 10

unsigned long MotorTimer = 0;
unsigned long brakeTimer = 0;

bool brakeIsOn = false;

/* cached settings from last update of remote */
#define kFORWARD 1
#define kBACKWARD -1
#define kZERO 0
int8_t fb_cache = kFORWARD; // forward/backward setting
int8_t p_cache = 0; // percentage motor
bool slowMode = true; // always start in slow mode

/* last set values for the motor speed targets */
int8_t next_left_motor = 0; // current setting, -10 to +10
int8_t next_right_motor = 0; // current setting, -10 to +10

int16_t current_left_target = 0;  // -MAXLEFTMOTOR .. MAXLEFTMOTOR (where we *want* to be)
int16_t current_left_motor = 0;   // -MAXLEFTMOTOR .. MAXLEFTMOTOR (what the motor is doing)
int16_t current_right_target = 0;
int16_t current_right_motor = 0;
bool decelLeft = false;
bool decelRight = false;

#define MINLEFTMOTOR  1000
#define MAXLEFTMOTOR  1150
#define MAXFASTLEFTMOTOR 1300
#define MINRIGHTMOTOR 1000
#define MAXRIGHTMOTOR 1150
#define MAXFASTRIGHTMOTOR 1300

// FIXME: make left/right
#define MAXTURNSPEED 1050

#define ACCEL 10
#define STOPPEDACCEL 40 // necessary to give the controllers time between fwd and rev
#define DECEL 200
#define MINBRAKEVAL 250 // go to full-stop-zero when we're below this value

void timerOneInterrupt()
{
  static uint8_t ctr = 0;
  if (++ctr != 0)
    return;
  
  // If the brakes are on, then shut down the motors.
  if (brakeIsOn) {
    current_left_motor = current_right_motor = 0;
    return;
  }
  
  // Perform acceleration and deceleration periodically (when the timer fires)
  int16_t maxLeftSpeed = slowMode ? MAXLEFTMOTOR : MAXFASTLEFTMOTOR;
  int16_t maxRightSpeed = slowMode ? MAXRIGHTMOTOR : MAXFASTRIGHTMOTOR;
  if (sign(current_left_target) != sign(current_right_target)) {
    // TURNING.
    maxLeftSpeed = maxRightSpeed = MAXTURNSPEED;
  }

  current_left_motor = performAccelerationWithConstraints(current_left_motor, current_left_target, MINLEFTMOTOR, maxLeftSpeed, &decelLeft);
  current_right_motor = performAccelerationWithConstraints(current_right_motor, current_right_target, MINRIGHTMOTOR, maxRightSpeed, &decelRight);
}

int16_t performAccelerationWithConstraints(int16_t motor, int16_t target, int16_t minVal, int16_t maxVal, bool *isDecelOut)
{
  // Constrain before we start
  if (abs(target) <= minVal) {
    target = 0;
  }

  // If we reached the minimum value, then we're stopped.
  if (target == motor && abs(target) == minVal) {
    *isDecelOut = false;
    //motor = 0;
    return 0;
  }
  
  // If we're accelerating, then do so SLOWLY.
  if (target >= 0 && motor >= 0 && target > motor) {
    // accelerating forward
    if (motor <= minVal)
      motor = min(motor + STOPPEDACCEL, target);
    else
      motor = min(motor + ACCEL, target);
    *isDecelOut = false;
  } else if (target < 0 && motor <= 0 && target < motor) {
    // accelerating backward
    if (motor >= -minVal)
      motor = min(motor - STOPPEDACCEL, target);
    else
      motor = min(motor - ACCEL, target);
    *isDecelOut = false;
  } else if (target < motor) {
    // slowing down from "too fast forward"
    motor = max(motor-DECEL, target);
    if (abs(motor) <= MINBRAKEVAL)
      motor = 0;
    *isDecelOut = true;
  } else if (target > motor) {
    // slowing down from "too fast backward"
    motor = min(motor + DECEL, target);
    if (abs(motor) <= MINBRAKEVAL)
      motor = 0;
    *isDecelOut = true;
  }

  // Constrain...
  if (abs(motor) > maxVal) {
    if (motor > 0)
      motor = maxVal;
    else
      motor = -maxVal;
  }

  return motor;
}
void setup()
{
  MCUSR = 0;  // clear out any flags of prior watchdog resets.

  // ensure the CS line is driven very early
  pinMode(MotorDACCSPin, OUTPUT);
  digitalWrite(MotorDACCSPin, HIGH); // active-low, so we want it high...
  delay(1000);


  pinMode(Motor1DirectionPin, INPUT);
  digitalWrite(Motor1DirectionPin, HIGH);
  pinMode(Motor2DirectionPin, INPUT);
  digitalWrite(Motor2DirectionPin, HIGH);

  pinMode(Motor1BrakePin, INPUT);
  digitalWrite(Motor1BrakePin, HIGH);
  pinMode(Motor2BrakePin, INPUT);
  digitalWrite(Motor2BrakePin, HIGH);
  
  Serial.begin(9600); // Primarily for talking to the music board
#ifdef DEBUG
  Serial.println("Debug enabled");
#endif
  softSerial.begin(9600); // For talking with the shoulder motor controller

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

//  wdt_enable(WDTO_1S);

  updateMotors(); // force back to 0 if we oddly rebooted

//  Timer1.initialize(50000); // 0.05 second interrupt period
//  Timer1.attachInterrupt(timerOneInterrupt);
}

inline int8_t sign(int16_t v)
{
  if (v < 0)
    return kBACKWARD;

  if (v > 0)
    return kFORWARD;

  return kZERO;
}

// Is it ok for us to set the motor to 'wantSetTo' if it was last set to 'lastSetTo'?
// return true if so, or false to abort.
bool sanityCheckMotor(int16_t lastSetTo, int16_t wantSetTo)
{
  int signLast = sign(lastSetTo);
  int signWant = sign(wantSetTo);

  // Anything is ok if it was off to begin with. (FIXME: don't try to go too fast.)
  if (signLast == kZERO)
    return true;

  // If we want to stop, that's okay.
  if (signWant == kZERO)
    return true;

  // Otherwise, expect that we're going in the same direction as we want to.
  if (signLast != signWant)
    return false;

  return true;
}

void updateMotors()
{
  static int16_t lastLeftMotor = 0;
  static int16_t lastRightMotor = 0;

  if (!sanityCheckMotor(lastLeftMotor, current_left_motor) ||
      !sanityCheckMotor(lastRightMotor, current_right_motor)) {
#ifdef DEBUG
    Serial.print(current_left_motor);
    Serial.print(" ");
    Serial.print(lastLeftMotor);
    Serial.println(" ACCEL ERR");
#endif
    current_left_target = current_right_target = 0;
    MotorDAC.setValue(0, LEFTDAC);
    MotorDAC.setValue(0, RIGHTDAC);
    return; // refuse to perfrom the update; shut down both motors instead.
  }

  setBrake(true, decelLeft);
  setBrake(false, decelRight);
  
  if (current_left_motor != lastLeftMotor) {
    lastLeftMotor = current_left_motor;
    if (sign(lastLeftMotor) == kFORWARD) {
      pinMode(Motor1DirectionPin, INPUT);
      digitalWrite(Motor1DirectionPin, HIGH);
    } else {
      pinMode(Motor1DirectionPin, OUTPUT);
      digitalWrite(Motor1DirectionPin, LOW); // drag to ground
    }
    MotorDAC.setValue(abs(lastLeftMotor), LEFTDAC);
  }
  if (current_right_motor != lastRightMotor) {
    lastRightMotor = current_right_motor;
    if (sign(lastRightMotor) == kFORWARD) {
      pinMode(Motor2DirectionPin, INPUT);
      digitalWrite(Motor2DirectionPin, HIGH);
    } else {
      pinMode(Motor2DirectionPin, OUTPUT);
      digitalWrite(Motor2DirectionPin, LOW); // drag to ground
    }
    MotorDAC.setValue(abs(lastRightMotor), RIGHTDAC);
  }
}

// Input l/r: [-10 .. +10]
void setMotorTargets(int l, int r)
{
  current_left_target = map(abs(l), 0, 10, MINLEFTMOTOR, slowMode ? MAXLEFTMOTOR : MAXFASTLEFTMOTOR);
  if (current_left_target <= MINLEFTMOTOR)
    current_left_target = 0;
  current_left_target = constrain(current_left_target, MINLEFTMOTOR, slowMode ? MAXLEFTMOTOR : MAXFASTLEFTMOTOR);

  if (l < 0) {
    current_left_target = -current_left_target;
  }

  current_right_target = map(abs(r), 0, 10, MINRIGHTMOTOR, slowMode ? MAXRIGHTMOTOR : MAXFASTRIGHTMOTOR);
  if (current_right_target <= MINRIGHTMOTOR)
    current_right_target = 0;
  current_right_target = constrain(current_right_target, MINRIGHTMOTOR, slowMode ? MAXRIGHTMOTOR : MAXFASTRIGHTMOTOR);

  if (r < 0) {
    current_right_target = -current_right_target;
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

  timerOneInterrupt();

  // set the motor DACs based on the current speed, if it changed since last loop (e.g. from the timer)
  updateMotors();
    
  /* Spin down the motors if their timers have expired */
  if (MotorTimer && MotorTimer < millis()) {
    MotorTimer = 0;
    setMotorTargets(0, 0);
  }

  /* Deal with brakes the same way */
  if (brakeTimer && brakeTimer < millis()) {
    brakeTimer = 0;
    setBrake(false); // turn off the brake
  }
  
  /* See if we have new RF commands waiting to be received - make sure timer doesn't go off while we do */

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
          //ForceRestart();
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
          softSerial.write('L');
          break;
        case ')':
          softSerial.write('R');
          break;
  
        case '^':
          softSerial.write('+');
          break;
        case 'v':
          softSerial.write('-');
          break;
          
        case 'M':
          wantStartMusic=true;
          break;
        case 'm':
          startMusic( MUSIC_NONE );
          SetTimer(&brakeTimer);
          setBrake(true);
          break;
      }
    }
  }
}

// These are individual brake lines for L/R.
void setBrake(bool leftBrake, bool isOn)
{
  if (leftBrake) {
    if (isOn) {
      pinMode(Motor1BrakePin, OUTPUT);
      digitalWrite(Motor1BrakePin, LOW);
    } else {
      pinMode(Motor1BrakePin, INPUT);
      digitalWrite(Motor1BrakePin, HIGH);
    }
  } else {
    if (isOn) {
      pinMode(Motor2BrakePin, OUTPUT);
      digitalWrite(Motor2BrakePin, LOW);
    } else {
      pinMode(Motor2BrakePin, INPUT);
      digitalWrite(Motor2BrakePin, HIGH);
    }
  }
}

// This is the emergency "all stop" brake.
void setBrake(bool isOn) {
  brakeIsOn = isOn;
  if (brakeIsOn) {
    setBrake(true, true);
    setBrake(false, true);
  } else {
    setBrake(true, false);
    setBrake(false, false);
  }
}

void ForceRestart()
{
#ifdef DEBUG
  Serial.println("Forcing WDT restart...");
#endif
  wdt_enable(WDTO_15MS);
  while(1) ;
}

