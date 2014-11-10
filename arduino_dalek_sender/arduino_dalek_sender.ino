#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/pgmspace.h>

#define RF_NODEID 1
#define RF_GROUPID 212

#define LED_PIN 6
#define SpeedSwitch 5
 #define kSlowMode 1
 #define kFastMode 0

const int leftButtonPin = 3;
const int rightButtonPin = 4;

// Constants returned by Handle* functions to denote what they did.
// These are bit-fields (1,2,4,8...)
#define DID_NOTHING 0
#define DID_SOMETHING 1
#define DID_OVERRIDE_JOYSTICK 2

#define RF_BROADCASTID 0

// SCALE_FACTOR is the percentage of full speed we want to operate at
#define SCALE_FACTOR 1.0

#define kFORWARD 0
#define kBACKWARD 1

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

/* LED pulsing variables */
int led_brightness = 255;
int led_direction = -5;
unsigned long led_timer = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing RFM12B");
  rf12_initialize(RF_NODEID, RF12_433MHZ, RF_GROUPID);
  Serial.println("Initialized sender.");
  
  pinMode(leftButtonPin, INPUT);
  pinMode(rightButtonPin, INPUT);
  
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH); // fake +5v for joystick
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW); // fake GND for joystick
  
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 100);
  pinMode(SpeedSwitch, INPUT);
  digitalWrite(SpeedSwitch, HIGH); // pull-up enabled
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
  
  if (*l_motor > 100)
    *l_motor = 100;
  if (*l_motor < -100)
    *l_motor = -100;
  if (*r_motor > 100)
    *r_motor = 100;
  if (*r_motor < -100)
    *r_motor = -100;
  
}

uint8_t ReadJoystick(float *x, float *y)
{
  int lrAnalog = analogRead(A0);
  int udAnalog = analogRead(A1);

  *x = *y = 0;
  
  /* Manual constants from tuning the joystick:
   * 
   * Full left: 174
   * Somewhat left: 347
   * center L/R: 492-540
   * somewhat right: 713
   * Full right: 814
   *
   * Full up: 156
   * Somewhat up: 300
   * Center U/D: 420-521
   * Somewhat down: 650
   * Full down: 797
   */

  /* If the joystick is centered, then return. */
  if (lrAnalog >= 492 && lrAnalog <= 540 &&
      udAnalog >= 420 && udAnalog <= 521) {
        // Dead stop: nothing to do
        return DID_NOTHING;
  }
  
  /* calculate joystick position, in percentages. */
  float joyX = 0, joyY = 0;
  if (lrAnalog < 492) {
    joyX = - (1.0 - (((float)lrAnalog - 174.0) / 492.0));
    if (joyX < -1) 
      joyX = -1;
  } else if (lrAnalog > 540) {
    joyX = ((float)lrAnalog - 540.0) / (814.0 - 540.0);
    if (joyX > 1)
      joyX = 1;
  }
  
  if (udAnalog < 420) {
    joyY = 1.0 - (((float)udAnalog - 156.0) / 420.0);
    if (joyY > 1)
      joyY = 1;
  } else if (udAnalog > 521) {
    joyY = - ((float)udAnalog - 521.0) / (797.0 - 521.0);
    if (joyY < -1)
      joyY = -1;
  }

  *x = joyX;
  *y = joyY;
  return DID_SOMETHING;
}

int HandleJoystick()
{
  float joyX, joyY;
  if (ReadJoystick(&joyX, &joyY) == DID_NOTHING) {
    // Dead stop; nothing to do
    return DID_NOTHING;
  }

  /* convert that to a polar coordinate - angle and distance. */
  float distance = sqrt(joyX*joyX + joyY*joyY) * 100.0 * SCALE_FACTOR;
  float angle = degrees(atan2(joyY, joyX));
  
   /* Determine what the new lm_state and rm_state should be. */
   int new_leftmotor;
   int new_rightmotor;
   
   OptimalThrust(angle, distance, &new_leftmotor, &new_rightmotor);
   
   if (digitalRead(SpeedSwitch) == kSlowMode) {
     // Slow mode: decrease motor speed by half
     new_leftmotor *= 2;
     new_leftmotor /= 3;
     new_rightmotor *= 2;
     new_rightmotor /= 3;
   }
   
   /* Compare the new states with the last sent states. If they differ, then send the new states. */
   int new_lm_state = new_leftmotor / 10;
   int new_rm_state = new_rightmotor / 10;

    target_lm_state = new_lm_state;
    target_rm_state = new_rm_state;
   
   return DID_SOMETHING;
}

void HandleMusic()
{
  // Read from the joystick what we want to do
  float joyX, joyY;
  ReadJoystick(&joyX, &joyY);

  if (joyX > 0.6 && abs(joyY) < 0.6) {
    // right
    if (digitalRead(SpeedSwitch) == kSlowMode)
      rf_send("M4", 2);
    else
      rf_send("M1", 2);
  } else if (joyX < -0.6 && abs(joyY) < 0.6) {
    // left
    if (digitalRead(SpeedSwitch) == kSlowMode)
      rf_send("M5", 2);
    else
      rf_send("M2", 2);
  } else if (abs(joyX) < 0.6 && joyY > 0.6) {
    // up
    if (digitalRead(SpeedSwitch) == kSlowMode)
      rf_send("M6", 2);
    else
      rf_send("M3", 2);
  } else if (abs(joyX) < 0.6 && joyY < -0.6) {
    // down
    rf_send('m'); // stop music
  }
}

int HandleShoulder()
{
  int leftState = digitalRead(leftButtonPin);
  int rightState = digitalRead(rightButtonPin);
   
   if (leftState && rightState) {
     // A double-press means we want to invoke music.
     HandleMusic();
     // Regardless of what that does, override the joystick.
     return DID_OVERRIDE_JOYSTICK;
   }
   
   if (leftState) {
     rf_send('(');
     return DID_SOMETHING;
   } else if (rightState) {
     rf_send(')');
     return DID_SOMETHING;
   }
   
   return DID_NOTHING;
}

void loop()
{
  // Handle motor ramp up/down
  if (millis() >= last_motor_update_time + 40) {

    // Once every 40ms, move 10% toward our goal speeds for both motors
      if (lm_state != target_lm_state) {
        int new_lm_state;
        if (target_lm_state > lm_state) {
          new_lm_state = lm_state + 1;
        } else {
          new_lm_state = lm_state - 1;
        }
        SendNewState('L', new_lm_state);
        lm_state = new_lm_state;
      }
      if (rm_state != target_rm_state) {
        int new_rm_state;
        if (target_rm_state > rm_state) {
          new_rm_state = rm_state + 1;
        } else {
          new_rm_state = rm_state - 1;
        }
        SendNewState('R', new_rm_state);
        rm_state = new_rm_state;
      }

    last_motor_counter++;

    if ((last_motor_counter & 1) == 1) {
      // Once every 80ms, if either motor should be moving, pulse the drives
      if (lm_state || rm_state) {
        rf_send('G');
      }
    }
    
    last_motor_update_time = millis();
  }

  if (millis() >= led_timer) {
    led_brightness += led_direction;
    if (led_brightness <= 0 || led_brightness >= 255) {
      led_direction = -led_direction;
    }
    analogWrite(LED_PIN, led_brightness);
    led_timer = millis() + 30;
  }

  int didSend = HandleShoulder();
  if (!(didSend & DID_OVERRIDE_JOYSTICK)) {
    didSend |= HandleJoystick();
  }
  
  if (didSend) {
    delay(15);
    led_brightness = 255;
    led_direction = -5;
  }
}


void rf_send(const char *data, int datasize)
{
  while (!rf12_canSend()) {
    rf12_recvDone();
  }

  rf12_sendStart(2, data, datasize);
  rf12_sendWait(1);
}

void rf_send(const char data)
{
  while (!rf12_canSend()) {
    rf12_recvDone();
  }
  
  rf12_sendStart(2, &data, 1);
  rf12_sendWait(1);
}

