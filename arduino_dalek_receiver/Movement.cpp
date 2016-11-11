#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "Movement.h"

// at 930, I can get it to back up, but not turn-in-place.
// at 900, this works pretty well but is laggy getting in to reverse.
// at 920 it turns-in-place. Still laggy, but probably the best we'll get.
//   could try changing ACCEL, or have a separate forward/backward ACCEL to 
//   get over the reverse "hump" faster, but I don't know that it would work.
// Regardless: if we have appreciable backward speed and it tries to stop,
// the brakes are kinda too fast.
// also: the left joystick is sticking forward :/
#define LMINBACKMOTOR 920
#define LMINMOTOR 980
#define LMAXMOTOR 1200 // or 1300 for fast?
#define LMAXTURN 1060

#define RMINBACKMOTOR 920
#define RMINMOTOR 970
#define RMAXMOTOR 1200 // or 1300 for fast?
#define RMAXTURN 1060

#define MAXSAFETY 1300 // don't ever allow values over this

#define ACCEL 3
#define CATCHUP 5       // multiplier on top of ACCEL if one motor needs to catch the other
#define DECEL 150       // estimated rate at which we decelerate with brakes on
#define MINBRAKEVAL 250 // go to full-stop-zero when we're below this value

#define STOPPED_THRESHOLD 5 // how many cycles to remain @ 0 before moving

#ifdef _UNIX
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#define kFORWARD 1
#define kBACKWARD -1
#define kZERO 0

inline int8_t sign(int16_t v)
{
  if (v < 0)
    return kBACKWARD;

  if (v > 0)
    return kFORWARD;

  return kZERO;
}

// Given an angle (degrees, where 0 is right, 90 is fwd, -90 is back,
// 180 is left) and an acceleration (range 0..100; the distance of the
// polar coordinate, basically) return an optimal l_motor and r_motor
// (range -100..100) speed to achieve that goal
static void OptimalThrust(int degs, int accel, int *l_motor, int *r_motor)
{
  if (degs >= 0 && degs <= 90) {
    // between full right (0) and forward (90)
    *l_motor = accel;
    *r_motor = accel * sin(radians(2*degs - 90));
  } else if (degs < 0 && degs >= -90) {
    // between full right (0) and backward (-90)
    *r_motor = -accel;
    *l_motor = accel * sin(radians(2*degs + 90));
  } else if (degs > 90 && degs <= 180) {
    // between full forward (90) and full left (180)
    *r_motor = accel;
    *l_motor = accel * sin(radians(2*degs - 90));
  } else if (degs < -90 && degs >= -180) {
    // between full backward (-90) and full left (-180)
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

// Given an X/Y from a joystick [-100..100] where 
//  y==+100 is forward and y==-100 is back; and
//  x==+100 is right and x==-100 is left
// figure out how to make that best move the motors.
void Movement::JoystickTarget(float x, float y)
{
  // Turn x and y in to polar coordinates
  float distance = sqrt(x*x + y*y);
  float angle = degrees(atan2(y, x));

  int new_leftmotor = 0;
  int new_rightmotor = 0;

  OptimalThrust(angle, distance, &new_leftmotor, &new_rightmotor);

  target_l_pct = new_leftmotor;
  target_r_pct = new_rightmotor;
}

void Movement::GetMotorTargetPercentages(int8_t *left, int8_t *right)
{
  *left = target_l_pct;
  *right = target_r_pct;
}

void Movement::GetMotorTargetValues(int16_t *left, int16_t *right)
{
  *left = percentToMotorDriverValue(target_l_pct, target_l_pct != target_r_pct, kLEFT);
  *right = percentToMotorDriverValue(target_r_pct, target_l_pct != target_r_pct, kRIGHT);
}

void Movement::GetMotorValues(int16_t *left, int16_t *right)
{
  *left = actual_l;
  if (actual_l >= 0 && actual_l <= LMINMOTOR) {
    *left = 0;
  }

  *right = actual_r;
  if (actual_r >= 0 && actual_r <= RMINMOTOR) {
    *right = 0;
  }

  // For reverse, return all the weird intermediate values...
}

void Movement::GetMotorDirection(int8_t *left, int8_t *right)
{
  *left = sign(actual_l);
  *right = sign(actual_r);
}

// Get the percentage of each motor's current state as a ratio of (L/R).
// (Return NAN if R is zero.)
float Movement::GetMotorRatio()
{
  if (abs(actual_r) <= RMINMOTOR)
    return NAN;

  float a = abs(actual_l) - (actual_l < 0 ? LMINBACKMOTOR : LMINMOTOR);
  float b = abs(actual_r) - (actual_r < 0 ? RMINBACKMOTOR : RMINMOTOR);

  return a / b;
}


void Movement::Brake(bool set)
{
  brakeEnabled = set;
}

// This is the global brake OR that we're decelerating on that motor...
void Movement::GetBrakeState(bool *left, bool *right) 
{
  *left = decelLeft || brakeEnabled;
  *right = decelRight || brakeEnabled;
}

void Movement::Reset()
{
  target_l_pct = target_r_pct = 0;
  actual_l = actual_r = 0;
  brakeEnabled = false;
  decelLeft = decelRight = false;
}

// Called regularly to update our state...
void Movement::Update()
{
  // If the global brake is on, then we don't move. Period.
  if (brakeEnabled) {
    actual_l = actual_r = 0;
    return;
  }

  bool canMoveL = true;
  bool canMoveR = true;
  // Motors must stop @ 0 for a given amount of time before moving again.
  if (actual_l == 0) {
    if (++stopped_l_time < STOPPED_THRESHOLD) {
      canMoveL = false;
      decelLeft = false; // brake has to be off...
    }
  } else {
    stopped_l_time = 0;
  }

  if (actual_r == 0) {
    if (++stopped_r_time  < STOPPED_THRESHOLD) {
      canMoveR = false;
      decelRight = false; // brake has to be off...
    }
  } else {
    stopped_r_time = 0;
  }

  // If we are turning - the motors don't match target values - then 
  // our max is slower, for reasons of control.
  bool isTurning = false;
  if (target_l_pct != target_r_pct) {
    isTurning = true;
  }

  // prepare new actual L/R values...
  int16_t l = actual_l;
  int16_t r = actual_r;

  // From the current motor states, determine how to best accelerate
  // to match the desired course. The default is 1/1 (which will be
  // re-attained when the ratio matches in Update()).

  int16_t lt = percentToMotorDriverValue(target_l_pct, isTurning, kLEFT);
  int16_t rt = percentToMotorDriverValue(target_r_pct, isTurning, kRIGHT);
  float oldRatio = GetMotorRatio();
  float newRatio;
  if (target_r_pct == 0) 
    newRatio = NAN;
  else
    newRatio = target_l_pct / target_r_pct;

  // By default, assume we're going to accelerate at some preset speed...
  int8_t accelLeft = ACCEL;
  int8_t accelRight = ACCEL;

  int16_t leftCorrection = MotorDifferenceMagnitude(actual_l, lt, kLEFT);
  int16_t rightCorrection = MotorDifferenceMagnitude(actual_r, rt, kRIGHT);

  // FIXME: handle all the NAN cases in oldRatio != newRatio.
  // ... hmm, or just use this?
  if (leftCorrection != rightCorrection) {
    // ... and if we're trying to move the motors at a different set of 
    // relative speeds, then figure out which motor needs to move faster to 
    // catch up, and do it
    
    if (rightCorrection > leftCorrection)
      accelRight = CATCHUP * ACCEL;
    else
      accelLeft = CATCHUP * ACCEL;
  }

  if (canMoveL) {
    l = performAccelerationWithConstraints(l, 
					   lt,
					   isTurning ? LMAXTURN : LMAXMOTOR,
					   accelLeft,
					   &decelLeft,
					   kLEFT);
  }

  if (canMoveR) {
    r = performAccelerationWithConstraints(r, 
					   rt,
					   isTurning ? RMAXTURN : RMAXMOTOR,
					   accelRight,
					   &decelRight,
					   kRIGHT);
  }

  // Set the new actual values

  actual_l = l;
  actual_r = r;
}

int16_t Movement::percentToMotorDriverValue(int8_t pct, 
					    bool isTurning, 
					    int8_t whichMotor)
{
  if (abs(pct) < 1)
    return 0;

  int16_t maxVal = (whichMotor == kLEFT ? LMAXMOTOR : RMAXMOTOR);
  if (isTurning)
    maxVal = (whichMotor == kLEFT ? LMAXTURN : RMAXTURN);

  int16_t ret = map(abs(pct), 1, 100, 
		    whichMotor == kLEFT ? LMINMOTOR : RMINMOTOR, maxVal);

  if (ret >= 0 && ret <= (whichMotor == kLEFT ? LMINMOTOR : RMINMOTOR))
    ret = kLEFT ? LMINMOTOR : RMINMOTOR;

  if (ret < 0 && ret > (whichMotor == kLEFT ? -LMINBACKMOTOR : -RMINBACKMOTOR))
    ret = (whichMotor == kLEFT ? LMINBACKMOTOR : RMINBACKMOTOR); // sign changed below

  if (ret >= maxVal)
    ret = maxVal;

  if (pct < 0)
    ret = -ret;

  return ret;
}

int16_t Movement::performAccelerationWithConstraints(int16_t current,
						     int16_t target,
						     int16_t maxVal,
						     int8_t accel,
						     bool *isDecelOut,
						     int8_t whichMotor)
{
  // Don't allow targets > MAXSAFETY. We do allow targets > maxVal,
  // which may or may not be a troublesome bug. It's important that we
  // allow current to be > maxVal -- we might have had a different
  // maxVal and we've dropped it, in which case we'll be decelerating
  // normally to attain it.

  // min value depends on direction...
  int16_t minVal = (whichMotor == kLEFT ? LMINMOTOR : RMINMOTOR);
  if (current < 0 ||
      (current == 0 && target < 0)) {
    minVal = (whichMotor == kLEFT ? LMINBACKMOTOR : RMINBACKMOTOR);
  }

  if (abs(target) > MAXSAFETY) {
    target = (target > 0 ? MAXSAFETY : -MAXSAFETY);
  }

  // stopped, and stopping?
  if (current == 0 && target == 0) {
    *isDecelOut = false;
    return 0;
  }

  // Reached our target? Brakes off, keep going...
  if (target == current && target >= minVal && target <= maxVal) {
    *isDecelOut = false;
    return target;
  }

  // accelerating: do it slowly...
  if (target >= 0 & current >= 0 && target > current) {
    // accelerating forward. From [0..minVal] we "accelerate" at a faster 
    // rate than when we're actually moving, because minVal is the minimum 
    // value required to actually make the motor move. We use that window 
    // as a buffer to give us some time for the motors to actually stop 
    // before reversing direction.
    if (current < minVal)
      current = minVal; // jump the gap
    else
      current = min(current + accel, target);
    *isDecelOut = false;
  } else if (target < 0 && current <= 0 && target < current) {
    // accelerating backward
    if (current > -minVal) {
      current = -minVal; // jump the gap
    }
    else
      current = max(current - accel, target);
    *isDecelOut = false;
  } else if (target < current) {
    // slowing down from "too fast forward"
    current = max(current-DECEL, target);
    // At some point we hit some minimum "close enough to zero" and we
    // just stop.
    if (abs(current) <= MINBRAKEVAL) {
      current = 0;
    }
    *isDecelOut = true;
  } else if (target > current) {
    // slowing down from "too fast backward"
    current = min(current + DECEL, target);
    if (abs(current) <= MINBRAKEVAL)
      current = 0;
    *isDecelOut = true;
  }

  // Ensure constraints
  if (abs(current) > MAXSAFETY) {
    current = (current > 0 ? MAXSAFETY : -MAXSAFETY);
  }

  if (abs(current) < minVal) {
    current = 0;
  }

  return current;
}

int16_t Movement::MotorDifferenceMagnitude(int16_t a, int16_t b, int8_t whichMotor)
{
  // return the absolute difference between the two - skipping the dead zone.
  int16_t minV = (whichMotor == kLEFT ? LMINMOTOR : RMINMOTOR);

  if (a > 0 && a >= minV) {
    a -= minV;
  } else if (a < 0 && a <= -minV) {
    a += minV;
  }
  if (b >= 0 && b >= minV) {
    b -= minV;
  } else if (b < 0 && b <= -minV) {
    b += minV;
  }

  return abs(a-b);
}

