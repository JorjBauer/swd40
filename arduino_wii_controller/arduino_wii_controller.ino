#include <Wire.h>
#include <JeeLib.h>
#include "WiiClassic.h"

#define RF_NODEID 1
#define RF_GROUPID 212

#define LED_PIN 6

WiiClassic ctrl = WiiClassic();
int lcx, lcy, rcx, rcy;
// How wide is each half of the left joystick's "center/NULL" range?
#define LEFT_CENTER_RANGE 10
// Same with the right joystick (which has a different precision)
#define RIGHT_CENTER_RANGE 5

#define kFORWARD 0
#define kBACKWARD 1

// #define DEBUG

int slow_mode = 1; // Slow or fast mode? Slow by default.

/* Protocol states initialized to invalid values */
int fb_state = -1;  // forward/backward state; 0/1
int p_state = -1;   // motor percentage; 0 to 10

/* Remote motor states (should they be pulsed right now). Again, init'd to invalid values. */
int lm_state = 0; // -10 to +10
int rm_state = 0; // -10 to +10

/* Vars to slowly ramp motors up/down */
unsigned long last_motor_update_time = 0;
unsigned char last_motor_counter = 0;
int target_lm_state = 0;
int target_rm_state = 0;

unsigned long last_data_update = 0;

/* LED pulsing variables */
int led_brightness = 255;
int led_direction = -5;
unsigned long led_timer = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(57600);
  Serial.println("init");
#endif

  Wire.begin();
//  nunchuck_setpowerpins();
  ctrl.begin();
  ctrl.update();

  // Read the center positions of the joysticks on start-up
  lcx = ctrl.leftStickX();
  lcy = ctrl.leftStickY();
  rcx = ctrl.rightStickX();
  rcy = ctrl.rightStickY();

  rf12_initialize(RF_NODEID, RF12_433MHZ, RF_GROUPID);

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 100);

#ifdef DEBUG  
  Serial.print("Startup centering: left ");
  Serial.print(lcx);
  Serial.print(",");
  Serial.print(lcy);
  Serial.print("; right ");
  Serial.print(rcx);
  Serial.print(",");
  Serial.println(rcy);
#endif
}

void SendNewState(char motor, int state)
{
  int fb_want;

  /* Set the remote end cache for f/b and percent */
  if (state >= 0) {
    fb_want = kFORWARD;
  } else {
    fb_want = kBACKWARD;
  }

  int pct_want = abs(state);
  if (pct_want > 10) {
    pct_want = 10;
  }

  if (fb_want != fb_state) {
    rf_send(fb_want == kFORWARD ? 'F' : 'B');
    fb_state = fb_want;
  }
  if (pct_want != p_state) {
    rf_send('0' + pct_want);
    p_state = pct_want;
  }

  /* Set the appropriate motor to those settings */
  rf_send(motor);
}

int HandleSlowFastMode()
{
  if (ctrl.selectPressed()) { // "select" is also "-"
    slow_mode = 1;
    return 0; // nothing sent
  } else if (ctrl.startPressed()) { // "start" is also "+"
    slow_mode = 0;
    return 0; // we still didn't send anything to the remote end.
  }

  return 0; // nothing sent to remote end.
}

// ReadRightJoystick returns 0 if the joyistick is centered, and 1 if not.
// It puts the percentage of full x/y (from -1.0 to 1.0) in *x and *y.
uint8_t ReadRightJoystick(float *x, float *y)
{
  int lrAnalog = ctrl.rightStickX();
  int udAnalog = ctrl.rightStickY();

  *x = *y = 0;

  // This is the right joystick, so its range is [0,31]. We have a center 
  // position that we cached on startup, which is [rcx, rcy] and we know 
  // the range of the "dead" middle spot we want, which is RIGHT_CENTER_RANGE.
  //
  // Turn all of this in to a percentage of left (negative) or right (positive)
  // and up (positive) or down (negative), from -1.0 to 1.0.

  // If the joystick is dead center, then it's nothing.
  if (lrAnalog >= (rcx - RIGHT_CENTER_RANGE) &&
      lrAnalog <= (rcx + RIGHT_CENTER_RANGE) &&
      udAnalog >= (rcy - RIGHT_CENTER_RANGE) &&
      udAnalog <= (rcy + RIGHT_CENTER_RANGE)) {
    // Dead center; do nothing.
    return 0;
  }

  if (lrAnalog < rcx - RIGHT_CENTER_RANGE) {
    // Left-leaning
    float range = (rcx - RIGHT_CENTER_RANGE);
    *x = -(1.0 - ((float)lrAnalog / range));
  } else if (lrAnalog > rcx + RIGHT_CENTER_RANGE) {
    // Right-leaning
    float range = 31.0 - (rcx + RIGHT_CENTER_RANGE);
    *x = (((float)lrAnalog - ((float)rcx + (float)RIGHT_CENTER_RANGE) ) / range);
  }

  if (udAnalog < rcy - RIGHT_CENTER_RANGE) {
    // down-leaning
    float range = (rcy - RIGHT_CENTER_RANGE);
    *y =  -(1.0 - ((float)udAnalog / range));
 
  } else if (udAnalog > rcy + RIGHT_CENTER_RANGE) {
    float range = 31.0 - (rcy + RIGHT_CENTER_RANGE);
    *y = (float)(udAnalog - rcy - RIGHT_CENTER_RANGE) / range;
  }

  return 1;
}

// ... and the same for the Left, whose range is [0,63].
uint8_t ReadLeftJoystick(float *x, float *y)
{
  int lrAnalog = ctrl.leftStickX();
  int udAnalog = ctrl.leftStickY();

  *x = *y = 0;
  // Turn all of this in to a percentage of left (negative) or right (positive)
  // and up (positive) or down (negative), from -1.0 to 1.0.

  // If the joystick is dead center, then it's nothing.
  if (lrAnalog >= (lcx - LEFT_CENTER_RANGE) &&
      lrAnalog <= (lcx + LEFT_CENTER_RANGE) &&
      udAnalog >= (lcy - LEFT_CENTER_RANGE) &&
      udAnalog <= (lcy + LEFT_CENTER_RANGE)) {
    // Dead center; do nothing.
    return 0;
  }

  if (lrAnalog < lcx - LEFT_CENTER_RANGE) {
    // Left-leaning
    float range = (lcx - LEFT_CENTER_RANGE);
    *x = -(1.0 - ((float)lrAnalog / range));
  } else if (lrAnalog > lcx + LEFT_CENTER_RANGE) {
    // Right-leaning
    float range = 63.0 - (lcx + LEFT_CENTER_RANGE);
    *x = (((float)lrAnalog - ((float)lcx + (float)LEFT_CENTER_RANGE) ) / range);
  }

  if (udAnalog < lcy - LEFT_CENTER_RANGE) {
    // down-leaning
    float range = (lcy - LEFT_CENTER_RANGE);
    *y =  -(1.0 - ((float)udAnalog / range));
 
  } else if (udAnalog > lcy + LEFT_CENTER_RANGE) {
    float range = 63.0 - (lcy + LEFT_CENTER_RANGE);
    *y = (float)(udAnalog - lcy - LEFT_CENTER_RANGE) / range;
  }

  return 1;  
}



int HandleMovement()
{
  float x, y;
  ReadRightJoystick(&x, &y);
  int new_rightmotor = (int)(100.0 * y);

  ReadLeftJoystick(&x, &y);
  int new_leftmotor = (int)(100.0 * y);

  if (slow_mode) {
    new_leftmotor *= 2;
    new_leftmotor /= 3;
    new_rightmotor *= 2;
    new_rightmotor /= 3;
  }
  
  if (new_leftmotor == 0 && new_rightmotor == 0) {
    target_lm_state = target_rm_state = 0;
    return 0;
  }

  // Compare the new states with the last states. If they differ, then
  // send the new states to the other end.
  int new_lm_state = new_leftmotor / 10;
  int new_rm_state = new_rightmotor / 10;
  
  target_lm_state = new_lm_state;
  target_rm_state = new_rm_state;
  
  return 1;
}

int HandleShoulder()
{
  if (ctrl.leftStickX() < (lcx - LEFT_CENTER_RANGE)) {
    rf_send(')');
    return 1;
  } else if (ctrl.leftStickX() > (lcx + LEFT_CENTER_RANGE)) {
    rf_send('(');
    return 1;
  }

  return 0;
}

int HandleSounds()
{
  int ret = 0;

  // Sound-related buttons, in order of priority (only one will send at a time)

  if (ctrl.lzPressed() || ctrl.rzPressed()) {
    rf_send('m');
    // Force the next motor update to stop the motors, too
    target_lm_state = target_rm_state = 0;
    lm_state = rm_state = 1;
    ret = 1;
  } else if (ctrl.leftShoulderPressed()) {
    // firing noise
    rf_send("M4", 2);
    ret = 1;
  } else if (ctrl.rightShoulderPressed()) {
    // "Exterminate!"
    rf_send("M5", 2);
    ret = 1;
  } else if (ctrl.leftDPressed()) {
    rf_send("M6", 2);
    ret = 1;
  } else if (ctrl.rightDPressed()) {
    rf_send("M7", 2);
    ret = 1;
  } else if (ctrl.upDPressed()) {
    rf_send("M8", 2);
    ret = 1;
  } else if (ctrl.downDPressed()) {
    rf_send("M9", 2);
    ret = 1;
  } else if (ctrl.xPressed()) {
    // interstitial1
    rf_send("M0", 2);
    ret = 1;
  } else if (ctrl.yPressed()) {
    // interstitial2
    rf_send("M1", 2);
    ret = 1;
  } else if (ctrl.aPressed()) {
    // interstitial3
    rf_send("M2", 2);
    ret = 1;
  } else if (ctrl.bPressed()) {
    rf_send("M3", 2);
    // interstitial4
    ret = 1;
  }

  return ret;
}

void rf_send(const char *data, int datasize)
{
#ifdef DEBUG
  Serial.println(data);
#endif
  while (!rf12_canSend()) {
    rf12_recvDone();
  }

  rf12_sendStart(2, data, datasize);
  rf12_sendWait(1);
}

void rf_send(const char data)
{
#ifdef DEBUG
  Serial.println(data);
#endif
  while (!rf12_canSend()) {
    rf12_recvDone();
  }

  rf12_sendStart(2, &data, 1);
  rf12_sendWait(1);
}

void loop() {
  // Handle motor ramp up/down
  if (millis() >= last_motor_update_time + 40) {
    
    // Once every 40ms, move 10% toward our goal speeds for both motors
    
    if (lm_state != target_lm_state) {
      int new_lm_state = lm_state;
      if (target_lm_state > lm_state) {
        new_lm_state ++;
      } else {
        new_lm_state --;
      }
      SendNewState('L', new_lm_state);
      lm_state = new_lm_state;
    }
    if (rm_state != target_rm_state) {
      int new_rm_state = rm_state;
      if (target_rm_state > rm_state) {
        new_rm_state ++;
      } else {
        new_rm_state --;
      }
      SendNewState('R', new_rm_state);
      rm_state = new_rm_state;
    }

    // Once every 80ms, if either motor should be moving, pulse the drives
    last_motor_counter++;
    if ((last_motor_counter & 1) == 1) {
      if (lm_state || rm_state) {
        rf_send('G');
      }
    }
    
    last_motor_update_time = millis();
  }


  // Handle LED pulsing
  if (millis() >= led_timer) {
    led_brightness += led_direction * (1 + (1 - slow_mode)); // pulse faster in fast mode
    if (led_brightness <= 0 || led_brightness >= 255) {
      if (led_brightness < 0)
	led_brightness = 0;
      if (led_brightness > 255)
	led_brightness = 255;

      led_direction = -led_direction;
    }
    analogWrite(LED_PIN, led_brightness);
    led_timer = millis() + 30;
  }

  // Periodically update the data
  if (millis() >= last_data_update + 100) {
    // Read new values from the controller
    ctrl.update();
  
    int didSend = HandleMovement();
    didSend |= HandleSlowFastMode();
    didSend |= HandleShoulder();
    didSend |= HandleSounds();
  
    if (didSend) {
      led_brightness = 255;
      led_direction = -5;
    }
    last_data_update = millis();
  }

}
