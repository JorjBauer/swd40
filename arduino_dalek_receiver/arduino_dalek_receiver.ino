#include <Arduino.h>
#include <avr/pgmspace.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69
#include <avr/wdt.h>
#include <AH_MCP4921.h>

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
#define Motor1DACCSPin 4
#define Motor1DirectionPin 5 // low for "reverse"
#define Motor2DACCSPin 6
#define Motor2DirectionPin 7 // low for "reverse"

AH_MCP4921 Motor1DAC(Motor1DACCSPin); // CS pin (uses SPI library)
AH_MCP4921 Motor2DAC(Motor2DACCSPin); // CS pin (uses SPI library)

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
int slowMode = 1; // always start in slow mode

/* last set values for the motor speed targets */
int next_left_motor = 0; // current setting, -10 to +10
int next_right_motor = 0; // current setting, -10 to +10

void setup()
{
  MCUSR = 0;  // clear out any flags of prior watchdog resets.

  pinMode(Motor1DirectionPin, OUTPUT);
  digitalWrite(Motor1DirectionPin, HIGH); // pull-up enabled
  
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

  pinMode(shoulderLeftPin, OUTPUT);
  pinMode(shoulderRightPin, OUTPUT);
  pinMode(gunUpPin, OUTPUT);
  pinMode(gunDownPin, OUTPUT);
  pinMode(PowerRelayPin, OUTPUT);
  pinMode(Motor1DACCSPin, OUTPUT);
  pinMode(Motor2DACCSPin, OUTPUT);

  pinMode(9, OUTPUT); // LED debugging

  wdt_enable(WDTO_1S);
  
  digitalWrite(PowerRelayPin, LOW); // tell the controllers it's business time! (This is active-low)
}

// Input l/r: [-10 .. +10]
void setMotorTargets(int l, int r)
{
  // The two motor controllers don't perform identically; we'll need to map them individually.
long minValueLeft = 800;
  long minValueRight = 800;
  long maxValueLeft = (slowMode ? 1500 : 2500);
  long maxValueRight = (slowMode ? 1500 : 2500);
  int left_abs = map(abs(l), 0, 10, minValueLeft, maxValueLeft); // low-end cutoff is 1v (~800/4096); high point is ~3v (~2500/4096). DAC is 12-bit (0-to-4096)
  int right_abs = map(abs(r), 0, 10, minValueRight, maxValueRight);

#ifdef DEBUG
  static char buf[30];
  sprintf(buf, "%d %d => %c%d %c%d\n", l, r, l < 0 ? '-' : '+', left_abs, r <0 ? '-' : '+', right_abs);
  Serial.println(buf);
#endif



  if (l >= 0) {
    digitalWrite(Motor1DirectionPin, HIGH); // forward!
  } else {
    digitalWrite(Motor1DirectionPin, LOW); // backward!
  }
  if (r >= 0) {
    digitalWrite(Motor2DirectionPin, HIGH);
  } else {
    digitalWrite(Motor2DirectionPin, LOW);
  }
  if (left_abs <= minValueLeft) {
    left_abs = 0; // don't set to < 1v; set to 0v instead.
  }
  if (right_abs <= minValueRight) {
    right_abs = 0;
  }
  
  Motor1DAC.setValue(left_abs);
  Motor2DAC.setValue(right_abs);
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
#ifdef DEBUG
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
          // Note that we reverse the sense of the right motor here b/c brushless, clockwise, etc.
          MakeMotorsGo(next_left_motor, -next_right_motor);
          break;

        /* Slow/fast mode */
        case '+':
          slowMode = 0;
          break;
        case '-':
          slowMode = 1;
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

