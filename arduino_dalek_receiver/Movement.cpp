#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "Movement.h"

/* FIXMEs:
 *   re-implement fast mode/slow mode
 *   per-motor min/max may or may not be necessary?
 *   turning & turn speeds
 *   disable reverse-motor-on-turn behavior
 *   
 * Probable errors to figure out:
 *   - If we start turning but then head full-forward, the motors 
 *     remain unbalanced until they hit full. This makes the turn 
 *     significant, where it shouldn't be happening at all.
 *     We should have some sense of "the target motors are equal percentages,
 *     so make the current motor values the same"
 *   - When going from stop to full-back, we don't accelerate - we just go 
 *     right to full back. BUG.
 */

#define MINMOTOR 1000
#define MAXMOTOR 1150 // or 1300 for fast?
#define MAXTURN 1050

#define ACCEL 1
#define DECEL 150
#define MINBRAKEVAL 250 // go to full-stop-zero when we're below this value

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
// figure out how to make that best move the motors
void Movement::JoystickTarget(float x, float y)
{
  // Turn x and y in to polar coordinates
  float distance = sqrt(x*x + y*y);
  float angle = degrees(atan2(y, x));

  int new_leftmotor = 0;
  int new_rightmotor = 0;

  OptimalThrust(angle, distance, &new_leftmotor, &new_rightmotor);

  target_l = new_leftmotor;
  target_r = new_rightmotor;
}

void Movement::GetMotorTargetPercentages(int8_t *left, int8_t *right)
{
  *left = target_l;
  *right = target_r;
}

void Movement::GetMotorValues(int16_t *left, int16_t *right)
{
  *left = actual_l;
  if (abs(actual_l) <= MINMOTOR) {
    *left = 0;
  }

  *right = actual_r;
  if (abs(actual_r) <= MINMOTOR) {
    *right = 0;
  }
}

void Movement::GetMotorDirection(int8_t *left, int8_t *right)
{
  *left = sign(actual_l);
  *right = sign(actual_r);
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
  target_l = target_r = 0;
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

  // Perform acceleration and deceleration. When doing this, we want to 
  // make sure that the relative L/R ratio is in line with what we're 
  // trying to achieve.

  int16_t l = actual_l;
  int16_t r = actual_r;

  float actualRatio = 0;
  if (target_r) actualRatio = (float)target_l / (float)target_r;
  float foundRatio = 0;
  if (r) foundRatio = (float)l / (float)r;

  // If we are turning - the motors don't match target values - then 
  // our max is slower, for reasons of control.
  int16_t maxSpeed = MAXMOTOR;
  if (target_l != target_r) {
    maxSpeed = MAXTURN;
  }

  // If either of the motors is moving, but needs to be moving the other way,
  // then we'll head back to stopped.
#if 0
  if ((target_r > 0 && r < 0) ||
      (target_r < 0 && r > 0) ||
      (target_l > 0 && l < 0) ||
      (target_l < 0 && l > 0)) {
    l = performAccelerationWithConstraints(l, 
					   0,
					   MINMOTOR,
					   maxSpeed,
					   &decelLeft);
    
    r = performAccelerationWithConstraints(r, 
					   0,
					   MINMOTOR,
					   maxSpeed,
					   &decelRight);
  }

  // Try to keep the motors moving more or less at the same ratio that
  // they need to move in the right direction.
  
  else
#endif
 if (foundRatio == 0 ||
      fabsf(actualRatio-foundRatio) <= 0.2 ||
	   (target_l == target_r && target_l == 0)) {

    // If we're not moving at the moment, or if the ratio is close to correct,
    // then just accelerate normally

    l = performAccelerationWithConstraints(l, 
					   percentToMotorDriverValue(target_l),
					   MINMOTOR,
					   maxSpeed,
					   &decelLeft);
    
    r = performAccelerationWithConstraints(r, 
					   percentToMotorDriverValue(target_r),
					   MINMOTOR,
					   maxSpeed,
					   &decelRight);
  }

  else {
      // Always assume that we want to slow one of the motors down to 
      // compensate, rather than speeding one up to compensate.

      // Cases:
      // 1. Moving forward; right motor is faster than we want
      // 2. Moving forward; left motor is faster than we want
      // 3. Turning left/forward; left motor is faster than we want
      // 4. Turning right/forward; right motor is faster than we want
      if (foundRatio > actualRatio && target_l >= 0 && target_r >= 0) {
	float ideal_l = 0;
	if (actualRatio != 0) ideal_l  = (float)r / actualRatio;
	l -= DECEL;
	decelLeft = true;
	if (l <= ideal_l) l = ideal_l;
      }
      else if (foundRatio < actualRatio && target_l >= 0 && target_r >= 0) {
	float ideal_r;
	if (actualRatio != 0) ideal_r = (float)l / actualRatio;
	r -= DECEL;
	decelRight = true;
	if (r <= ideal_r) r = ideal_r;
      }

      // 5. Turning pure left or pure right, and the other motor is drifting

      // 6. Turning left/backward; left motor is faster than we want
      // 7. Turning right/backward; right motor is faster than we want
      // 8. Moving backward; right motor is faster than we want
      // 9. Moving backward; left motor is faster than we want

      else {
	l = performAccelerationWithConstraints(l, 
					       percentToMotorDriverValue(target_l),
					       MINMOTOR,
					       maxSpeed,
					       &decelLeft);
	
	r = performAccelerationWithConstraints(r, 
					       percentToMotorDriverValue(target_r),
					       MINMOTOR,
					       maxSpeed,
					       &decelRight);
	
      }
  }
  
  // Set the new actual values

  actual_l = l;
  actual_r = r;
}

int16_t Movement::percentToMotorDriverValue(int8_t pct)
{
  if (abs(pct) < 1)
    return 0;

  int16_t ret = map(abs(pct), 1, 100, MINMOTOR, MAXMOTOR);

  if (ret <= MINMOTOR)
    ret = MINMOTOR;

  if (ret >= MAXMOTOR)
    ret = MAXMOTOR;

  if (pct < 0)
    ret = -ret;

  return ret;
}

int16_t Movement::performAccelerationWithConstraints(int16_t current,
						     int16_t target,
						     int16_t minVal,
						     int16_t maxVal,
						     bool *isDecelOut)
{
  if (target > maxVal)
    target = maxVal;

  // accelerating: do it slowly...
  if (target >= 0 & current >= 0 && target > current) {
    // accelerating forward. From [0..minVal] we "accelerate" at a faster 
    // rate than when we're actually moving, because minVal is the minimum 
    // value required to actually make the motor move. We use that window 
    // as a buffer to give us some time for the motors to actually stop 
    // before reversing direction.
    
    if (current < minVal)
      current = max(current + 250 * ACCEL, minVal); // jump the gap *SLOWLY* so the motor controllers don't hate on us
    else
      current = min(current + ACCEL, target);
    *isDecelOut = false;
  } else if (target < 0 && current <= 0 && target < current) {
    // accelerating backward
    if (current > -minVal)
      current = max(current - 250 * ACCEL, -minVal); // jump the gap *SLOWLY* going in to reverse.
    else
      current = max(current - ACCEL, target);
    *isDecelOut = false;
  } else if (target < current) {
    // slowing down from "too fast forward"
    current = max(current-DECEL, target);
    // At some point we hit some minimum "close enough to zero" and we
    // just stop.
    if (abs(current) <= MINBRAKEVAL)
      current = 0;
    *isDecelOut = true;
  } else if (target > current) {
    // slowing down from "too fast backward"
    current = min(current + DECEL, target);
    if (abs(current) <= MINBRAKEVAL)
      current = 0;
    *isDecelOut = true;
  }

  // Ensure constraints
  if (abs(current) > maxVal) {
    current = (current > 0 ? maxVal : -maxVal);
  }

  return current;
}


