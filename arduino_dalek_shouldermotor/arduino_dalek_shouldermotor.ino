#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

#define shoulderLeftPin 5
#define shoulderRightPin 6

#define FLOAT_TIME 10

// M1 is connected to both motors for the shoulder
#define SHOULDER_TURN_SPEED 180

unsigned long ShoulderTimer = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing secondary motor shield...");
  md.init();
  Serial.println("Initialized.");
  
  pinMode(shoulderLeftPin, INPUT);
  pinMode(shoulderRightPin, INPUT);
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
  
  if (digitalRead(shoulderLeftPin) == HIGH) {
    SetTimer(&ShoulderTimer);
    md.setM1Speed(-SHOULDER_TURN_SPEED);
    didSomething = 1;
    Serial.println("L");
  }
  else if (digitalRead(shoulderRightPin) == HIGH) {
    SetTimer(&ShoulderTimer);
    md.setM1Speed(SHOULDER_TURN_SPEED);
    didSomething = 1;
    Serial.println("R");
  }
  
  if (didSomething)
    delay(10);
}


