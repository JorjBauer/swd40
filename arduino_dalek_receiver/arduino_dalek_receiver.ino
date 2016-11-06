#include <Arduino.h>
#include <avr/pgmspace.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69
#include <avr/wdt.h>
#include <AH_MCP4921.h>
#include <SoftwareSerial.h>

#include "Movement.h"

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

#define VPIN A7
#define VCUTOFF 780

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
 *  16/A2 Power switch LED
 *  17/A3 
 *  18/A4 DAC SCK
 *  19/A5 DAC SDI
 *  x /A6
 *  x /A7 Power in from 10k/82k voltage divider off of battery
 */

#define MotorDACCSPin 4
#define Motor1DirectionPin 5 // low for "reverse"
#define Motor2DirectionPin 7 // low for "reverse"
#define Motor1BrakePin 6 // low for "brake engaged"
#define Motor2BrakePin 3 // low for "brake engaged"

#define DAC_SDI_PIN A5
#define DAC_SCK_PIN A4

#define LED_PIN A2

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
unsigned long ledTimer = 0;

int led_brightness = 255;
int led_direction = -5;
bool blinkingLight = false;

Movement movement; // the movement control engine code...
bool brakeIsOn = false;

#define kFORWARD 1
#define kBACKWARD -1
#define kZERO 0

/* cached settings from last update of remote */
bool slowMode = true; // always start in slow mode

void timerOneInterrupt()
{
  static uint8_t ctr = 0;
  if (++ctr != 0)
    return;

  movement.Update();
}

void setup()
{
  MCUSR = 0;  // clear out any flags of prior watchdog resets.

  // ensure the CS line is driven very early
  pinMode(MotorDACCSPin, OUTPUT);
  digitalWrite(MotorDACCSPin, HIGH); // active-low, so we want it high...
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 128); // This isn't an LED; it's a 5v lamp. 127 is off. 128 is on.

  pinMode(Motor1DirectionPin, INPUT);
  digitalWrite(Motor1DirectionPin, HIGH);
  pinMode(Motor2DirectionPin, INPUT);
  digitalWrite(Motor2DirectionPin, HIGH);

  pinMode(Motor1BrakePin, INPUT);
  digitalWrite(Motor1BrakePin, HIGH);
  pinMode(Motor2BrakePin, INPUT);
  digitalWrite(Motor2BrakePin, HIGH);

  pinMode(VPIN, INPUT); // voltage input: 82k/10k voltage divider from the battery
  
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Debug enabled");
#else
  Serial.begin(9600); // Primarily for talking to the music board
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

  bool decelLeft, decelRight;
  movement.GetBrakeState(&decelLeft, &decelRight);
#ifdef DEBUG
  static bool lastDL = false;
  if (lastDL != decelLeft) {
    Serial.print("BL: ");
    Serial.println(decelLeft ? 1 : 0);
    lastDL = decelLeft;
  }
#endif
  setBrake(true, decelLeft);
  setBrake(false, decelRight);
  
  int16_t current_left_motor, current_right_motor;
  movement.GetMotorValues(&current_left_motor, 
        &current_right_motor);

  // The controllers need to be told to go in to reverse *before* we go in to reverse.
  // So it's possible that Movement will tell us we're moving @ 0, but that we also need 
  // to have reverse engaged - so that it happens before we start trying to move.
  int8_t currentLeftDir, currentRightDir;
  movement.GetMotorDirection(&currentLeftDir, &currentRightDir);
  
  if (!sanityCheckMotor(lastLeftMotor, current_left_motor) ||
      !sanityCheckMotor(lastRightMotor, current_right_motor)) {
#ifdef DEBUG
    Serial.println(" ACCEL ERR");
#endif
    movement.Reset();
    MotorDAC.setValue(0, LEFTDAC);
    MotorDAC.setValue(0, RIGHTDAC);
    return; // refuse to perfrom the update; shut down both motors instead.
  }
  
  static int8_t lastLeftMotorDir = kZERO;
  static int8_t lastRightMotorDir = kZERO;
  
  if (lastLeftMotorDir != currentLeftDir) {
    lastLeftMotorDir = currentLeftDir;
    if (lastLeftMotorDir == kBACKWARD) {
#ifdef DEBUG
      Serial.println(" rev");
#endif
      pinMode(Motor1DirectionPin, OUTPUT);
      digitalWrite(Motor1DirectionPin, LOW); // drag to ground
    } else {
#ifdef DEBUG
      Serial.println(" fwd");
#endif
      pinMode(Motor1DirectionPin, INPUT);
      digitalWrite(Motor1DirectionPin, HIGH);
    }
  }

  if (lastRightMotorDir != currentRightDir) {
    lastRightMotorDir = currentRightDir;
    if (lastRightMotorDir == kBACKWARD) {
      pinMode(Motor2DirectionPin, OUTPUT);
      digitalWrite(Motor2DirectionPin, LOW); // drag to ground
    } else {
      pinMode(Motor2DirectionPin, INPUT);
      digitalWrite(Motor2DirectionPin, HIGH);
    }
  }
  
  if (current_left_motor != lastLeftMotor) {
    lastLeftMotor = current_left_motor;
#ifdef DEBUG
    Serial.print("cl: ");
    Serial.print(current_left_motor);
    Serial.print(" ll: ");
    Serial.println(lastLeftMotor);
#endif
    MotorDAC.setValue(abs(lastLeftMotor), LEFTDAC);
  }
  if (current_right_motor != lastRightMotor) {
    lastRightMotor = current_right_motor;
    MotorDAC.setValue(abs(lastRightMotor), RIGHTDAC);
  }
}

void MakeMotorsGo(int8_t joystickX, int8_t joystickY)
{
  // debugging: flash LED when we get a G pulse
  static bool ledState = 0;
  ledState = !ledState;
  digitalWrite(9, ledState);

  movement.JoystickTarget(joystickX, joystickY);

  int8_t l, r;
  movement.GetMotorTargetPercentages(&l, &r);
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
#ifdef DEBUG
    Serial.println("-Motor");
#endif
    movement.JoystickTarget(0, 0);
  }

  /* Deal with brakes the same way */
  if (brakeTimer && brakeTimer < millis()) {
    brakeTimer = 0;
    setBrake(false); // turn off the brake
  }

  /* Blinky Blinky little light? */
  if (analogRead(VPIN) <= VCUTOFF) {
    blinkingLight = true;
  } else {
    blinkingLight = false;
    analogWrite(LED_PIN, 128); // This isn't an LED; it's a 5v lamp. 127 is off. 128 is on.
  }

  if (blinkingLight && millis() >= ledTimer) {
    led_brightness += led_direction;
    if (led_brightness <= 0 || led_brightness >= 255) {
      if (led_brightness < 0)
        led_brightness = 0;
      if (led_brightness > 255)
        led_brightness = 255;

      led_direction = -led_direction;
    }
    analogWrite(LED_PIN, led_brightness);
    ledTimer = millis() + 10; // FIXME: pulse faster/slower depending on voltage, maybe?
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
          
	/* Have a joystick position: set motor targets */
        case 'G':
          // We'll read the X/Y and then pulse...
          // FIXME: assumes data exists
          MakeMotorsGo(radio.DATA[i+1], radio.DATA[i+2]);
          i += 2;
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
  // tell the movement object we're in braking mode
  movement.Brake(isOn);
  // set the actual brake pins immediately
  setBrake(false, isOn);
  setBrake(true, isOn);

}

void ForceRestart()
{
#ifdef DEBUG
  Serial.println("Forcing WDT restart...");
#endif
  wdt_enable(WDTO_15MS);
  while(1) ;
}

