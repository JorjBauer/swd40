#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/pgmspace.h>

#define RF_NODEID 1
#define RF_GROUPID 212

const int leftButtonPin = 3;
const int rightButtonPin = 4;

#define RF_BROADCASTID 0

#define kLEFT 0
#define kRIGHT 1
#define kFORWARD 0
#define kBACKWARD 1

/* Protocol states initialized to invalid values */
int fb_state = -1;  // forward/backward state; 0/1
int p_state = -1;   // motor percentage; 0 to 10

/* Remote motor states (should they be pulsed right now). Again, init'd to invalid values. */
int lm_state = -9999; // -10 to +10
int rm_state = -9999; // -10 to +10

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


int HandleJoystick()
{
  int lrAnalog = analogRead(A0);
  int udAnalog = analogRead(A1);
  
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

  /* If the joystick is quiescent, then return. */
  if (lrAnalog >= 492 && lrAnalog <= 540 &&
      udAnalog >= 420 && udAnalog <= 521) {
        return 0;
  }
  
  /* calculate joystick position, in percentages. */
  float joyX = 0, joyY = 0;
  if (lrAnalog < 492) {
    joyX = - (((float)lrAnalog - 174.0) / 492.0);
    if (joyX < -1) 
      joyX = -1;
  } else if (lrAnalog > 540) {
    joyX = ((float)lrAnalog - 540.0) / (814.0 - 540.0);
    if (joyX > 1)
      joyX = 1;
  }
  
  if (udAnalog < 420) {
    joyY = - (((float)udAnalog - 156.0) / 420.0);
    if (joyY < -1)
      joyY = -1;
  } else if (udAnalog > 521) {
    joyY = ((float)udAnalog - 521.0) / (797.0 - 521.0);
    if (joyY > 1)
      joyY = 1;
  }
   
   /* Determine what the new lm_state and rm_state should be. */
   float new_leftmotor;
   float new_rightmotor;
   
    new_leftmotor = joyY;
    if (joyX > 0) {
      new_leftmotor *= (1.0 - joyX);
    }
     
    new_rightmotor = joyY;
    if (joyX < 0) {
      new_rightmotor *= (1.0 - (abs(joyX)));
    }
  
   /* Compare the new states with the last sent states. If they differ, then send the new states. */
   int new_lm_state = (int)(10.0 * (new_leftmotor + 0.05));
   int new_rm_state = (int)(10.0 * (new_rightmotor + 0.05));

   if (lm_state != new_lm_state) {
     SendNewState('L', new_lm_state);
     lm_state = new_lm_state;
   }
   
   if (rm_state != new_rm_state) {
     SendNewState('R', new_rm_state);
     rm_state = new_rm_state;
   }

  /* If we get here, the motors aren't quiescent, and we've told the remote end how we want them set; pulse them. */   
   rf_send('G'); // one pulse
   
   return 1; // we did something
}

int HandleShoulder()
{
  int leftState = digitalRead(leftButtonPin);
  int rightState = digitalRead(rightButtonPin);
   
   if (leftState && rightState) {
     // ignore double-presses.
     return 0;
   }
   
   if (leftState) {
     rf_send('(');
     return 1;
   } else if (rightState) {
     rf_send(')');
     return 1;
   }
   
   return 0;
}

void loop()
{
  int didSend = HandleJoystick();
  didSend |= HandleShoulder();
  
  if (didSend) {
    delay(100);
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

