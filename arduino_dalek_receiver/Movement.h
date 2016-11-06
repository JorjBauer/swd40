#include <stdio.h>

#ifdef _UNIX
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
#else
#include <Arduino.h>
#endif

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)


class Movement {
 public:
  Movement() { actual_l = actual_r = target_l = target_r = 0; brakeEnabled = false; decelLeft = decelRight = false; };
  ~Movement() {};

  void Update();
  void JoystickTarget(float x, float y);
  void GetMotorTargetPercentages(int8_t *left, int8_t *right);
  void GetMotorValues(int16_t *left, int16_t *right);
  void GetMotorDirection(int8_t *left, int8_t *right);
  void Brake(bool set);
  void GetBrakeState(bool *left, bool *right);

  void Reset(); // called if there's an error in the engine...

 protected:
  int16_t percentToMotorDriverValue(int8_t pct);
  int16_t performAccelerationWithConstraints(int16_t current,
					     int16_t target,
					     int16_t minVal,
					     int16_t maxVal,
					     bool *isDecelOut);

 private:
  int8_t target_l, target_r; // percentages, -100..100
  int16_t actual_l, actual_r; // motor driver values, -MAXMOTOR..MAXMOTOR
  bool brakeEnabled;
  bool decelLeft, decelRight; // whether or not we're intentionally decelerating each motor
};
