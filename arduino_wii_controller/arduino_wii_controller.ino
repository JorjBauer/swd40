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

int slow_mode = 1; // Slow or fast mode? Slow by default.

/* Protocol states initialized to invalid values */
int fb_state = -1;  // forward/backward state; 0/1
int p_state = -1;   // motor percentage; 0 to 10

/* Remote motor states (should they be pulsed right now). Again, init'd to invalid values. */
int lm_state = -9999; // -10 to +10
int rm_state = -9999; // -10 to +10

/* LED pulsing variables */
int led_brightness = 255;
int led_direction = -5;
unsigned long led_timer = 0;

void setup() {
  Wire.begin();
  nunchuk_setpowerpins();
  ctrl.begin();
  ctrl.update();

  rf12_initialize(RF_NODEID, RF12_433MHZ, RF_GROUPID);

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 100);

  // Read the center positions of the joysticks on start-up
  lcx = ctrl.leftStickX();
  lcy = ctrl.leftStickY();
  rcx = ctrl.rightStickX();
  rcy = ctrl.rightStickY();
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

// Given an angle (degrees, where 0 is straight forward?) and an acceleration
// (range 0..100; the distance of the polar coordinate, basically) return
// an optimal l_motor and r_motor (range -100..100) speed to achieve that goal
void OptimalThrust(int degs, int accel, int *l_motor, int *r_motor)
{
  /*
  degs = ((degs + 180) % 360) - 180; // Normalize to [-180, 180)
  accel = min(max(0, accel), 100);         // Normalize to [0, 100]
  int v_a = accel * (45 - degs % 90) / 45;
  int v_b = min(100, 2 * r + v_a, 2 * r - v_a);
  if (degs < -90) {
  *l_motor = -v_b;
  *r_motor = -v_a;
  } else if (degs < 0) {
  *l_motor = -v_a;
  *r_motor = v_b;
  } else if (degs < 90) {
  *l_motor = v_b;
  *r_motor = v_a;
  } else {
  *l_motor = v_a;
  *r_motor = -v_b;
  }
  */

  /* The above might be more correct, but this is more legible: */

  if (degs >= 0 && degs <= 90) {
    *l_motor = accel;
    *r_motor = accel * sin(radians(2*degs - 90));
  } else if (degs < 0 && degs >= -90) {
    *r_motor = -accel;
    *l_motor = accel * sin(radians(2*degs + 90));
  } else if (degs > 90 && degs <= 180) {
    *r_motor = accel;
    *l_motor = accel * sin(radians(2*degs - 90));
  } else if (degs < -90 && degs >= -180) {
    *l_motor = -accel;
    *r_motor = accel * sin(radians(2*degs + 90));
  }

  /* Make sure to cap the motors at their peak values... */
  if (*l_motor > 100)
    *l_motor = 100;
  if (*l_motor < -100)
    *l_motor = -100;
  if (*r_motor > 100)
    *r_motor = 100;
  if (*r_motor < -100)
    *r_motor = -100;
}

int HandleSlowFastMode()
{
  if (ctrl.selectPressed()) { // "select" is also "-"
    slow_mode = 0;
    return 1;
  } else if (ctrl.startPressed()) { // "start" is also "+"
    slow_mode = 1;
    return 1;
  }

  return 0;
}

uint8_t ReadJoystick(float *x, float *y)
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
    *x = -((float)lrAnalog / range);
  } else if (lrAnalog > rcx + RIGHT_CENTER_RANGE) {
    // Right-leaning
    float range = 31.0 - (rcx + RIGHT_CENTER_RANGE);
    *x = (((float)lrAnalog - (float)rcx + (float)RIGHT_CENTER_RANGE ) / range);
  }

  if (udAnalog < rcy - RIGHT_CENTER_RANGE) {
    float range = (rcy - RIGHT_CENTER_RANGE);
    *y = ((float)udAnalog / range);
  } else if (udAnalog > rcy + RIGHT_CENTER_RANGE) {
    float range = 31.0 - (rcy + RIGHT_CENTER_RANGE);
    *y = -(((float)lrAnalog - (float)rcx + (float)RIGHT_CENTER_RANGE ) / range);
  }

  return 1;
}

int HandleMovement()
{
  float x, y;
  ReadJoystick(&x, &y);

  // Turn x and y in to polar coordinates
  float distance = sqrt(x*x + y*y);
  float angle = degrees(atan2(joyY, joyX));

  int new_leftmotor;
  int new_rightmotor;

  OptimalThrust(angle, distance, &new_leftmotor, &new_rightmotor);

  if (slowMode) {
    new_leftmotor /= 2;
    new_rightmotor /= 2;
  }

  // Compare the new states with the last states. If they differ, then
  // send the new states to the other end.
  int new_lm_state = new_leftmotor / 10;
  int new_rm_state = new_rightmotor / 10;

  if (lm_state != new_lm_state) {
    SendNewState('L', new_lm_state);
    lm_state = new_lm_state;
  }

  if (rm_state != new_rm_state) {
    SendNewState('R', new_rm_state);
    rm_state = new_rm_state;
  }

  // Motor settings are in sync now, so send a Go pulse
  rf_send('G');

  return 1;
}

int HandleShoulder()
{
  if (ctrl.leftStickX() < lcx - LEFT_CENTER_RANGE) {
    rf_send('(');
    return 1;
  } else if (ctrl.leftStickX() > lcx + LEFT_CENTER_RANGE) {
    rf_send(')');
    return 1;
  }

  return 0;
}

int HandleSounds()
{
  int ret = 0;

  // Sound-related buttons, in order of priority (only one will send at a time)

  if (ctrl.lzPressed() || ctrl.rzPressed()) {
    rf_send("m");
    ret = 1;
  } else if (ctrl.leftShoulderPressed()) {
    // firing noise
    rf_send("M4", 2);
    ret = 1;
  } else if (ctrl.rightShoulderPressed()) {
    // "Exterminate!"
    rf_send("M5", 2);
    ret = 1;
  } else if (ctrl.leftDPressed() || ctrl.rightDPressed() ||
	     ctrl.upDPressed() || ctrl.downDPressed()) {
    // theme music
    rf_send("M6", 2);
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

void loop() {
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

  // Read new values from the controller
  ctrl.update();

  int didSend = HandleMovement();
  didSend |= HandleSlowFastMode();
  didSend |= HandleShoulder();
  didSend |= HandleSounds();

  if (didSend) {
    delay(100);
    led_brightness = 255;
    led_direction = -5;
  } else {
    delay(20); // don't overload the Wii controller...
  }

}
