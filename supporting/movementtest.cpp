#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "Movement.h"

Movement m;

typedef struct _replayStep {
  float x;
  float y;
  unsigned long timeInMillis; // how long to hold this position
  const char *msg;
} replayStep;

#define REPLAYSTEPS 12
replayStep replay[REPLAYSTEPS] = { {0,      0,    100, "stopped"},
				   {0,    100,    135, "drive forward"},
				   {50,   100,     20, "bear right"},
				   {0,    100,    130, "back to full forward"},
				   {0,      0,     20, "stop"},
				   {0,   -100,    140, "full backward"},
				   {0,      0,     20, "stop"},
				   // test cases for ratio balancing:
				   // 1. Want to go full forward, but one motor is ahead of the other
				   //    start by turning left, then head full forward
				   {-100,   0,    150, "1a. left;"},
				   {0,    100,    150, "1b. full forward."},
				   // 2. Want to turn, but I'm going fully forward
				   { 100,   0,    150, "2a. right."},
				   // 3. Want to go full backward, but I'm turning
				   {   0,-100,    150, "3a. backward;"},
				   {0,      0,     20, "3b. stop."}

};

// delay -- but call the motor update while delaying
void delayMs(long millis)
{
  while (millis > 0) {
    m.Update();
    //    usleep(1000); // if we wanted to make this real-time output...
    millis -= 1;
    int8_t l, r;
    m.GetMotorTargetPercentages(&l, &r);
    float targetRatio = (float)l / (float)r;
    printf("%ld Left target: %d%%; right: %d%% [%f] ", millis, l, r, targetRatio);
    int16_t tl, tr;
    m.GetMotorTargetValues(&tl, &tr);
    printf("[%d / %d] ", tl, tr);

    int16_t cl, cr;
    m.GetMotorValues(&cl, &cr);
    bool bl, br;
    m.GetBrakeState(&bl, &br);
    // The actual ratio is weird, because there's a min and max value. We want 
    // the percentage of that, for each, compared as a ratio. Which is hard. 
    // So it's encapsulated inside the Movement class, b/c it's not simply 
    // (float) cl / (float) cr
    float actualRatio = m.GetMotorRatio();
    printf("Left motor: %d%c; Right:%d%c [%f]\n", cl, bl?'*':' ', cr, br?'*':' ', actualRatio);
  }
}

int main(int argc, char *argv[])
{
  for (int i=0; i<REPLAYSTEPS; i++) {
    replayStep s = replay[i];
    printf("'%s': Joystick moved to %f %f\n", s.msg, s.x, s.y);
    m.JoystickTarget(s.x, s.y);
    delayMs(s.timeInMillis);
  }
}


/*

Questions.

Does the joystick really go to +100,+100? If it did, that would be great - but 
given it's moving in a circle, I doubt it does. That might account for some of 
the oddities.

 */
