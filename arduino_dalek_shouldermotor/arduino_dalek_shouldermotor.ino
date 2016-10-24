#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

#define FLOAT_TIME 100

// M1 is connected to both motors for the shoulder
#define SHOULDER_TURN_SPEED 180
// M2 is connected to the motor for the gun stalk (or will be, some day, maybe)
#define GUN_SPEED 90

unsigned long ShoulderTimer = 0;
unsigned long GunTimer = 0;

void setup()
{
  Serial.begin(9600); // intentionally slow to minimize errors
  Serial.println("Initializing secondary motor shield...");
  md.init();
  Serial.println("Initialized.");
}

void SetTimer(unsigned long *t)
{
  *t = millis() + FLOAT_TIME;
}

void loop()
{
  int didSomething = 0;
  
  if (ShoulderTimer && ShoulderTimer < millis()) {
    ShoulderTimer = 0;
    md.setM1Speed(0);
  }
  if (GunTimer && GunTimer < millis()) {
    GunTimer = 0;
    md.setM2Speed(0);
  }

  if (Serial.available()) {
    uint8_t c = Serial.read();
    switch (c) {
      case 'L':
        SetTimer(&ShoulderTimer);
        md.setM1Speed(-SHOULDER_TURN_SPEED);
        didSomething = 1;
        break;
      case 'R':
        SetTimer(&ShoulderTimer);
        md.setM1Speed(SHOULDER_TURN_SPEED);
        didSomething = 1;
        break;
      case '+':
        SetTimer(&GunTimer);
        md.setM2Speed(GUN_SPEED);
        didSomething = 1;
        break;
      case '-':
        SetTimer(&GunTimer);
        md.setM2Speed(GUN_SPEED);
        didSomething = -1;
        break;
    }
  }
  
  if (didSomething)
    delay(10);
}


