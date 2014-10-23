#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/pgmspace.h>

#define RF_NODEID 1
#define RF_GROUPID 212

const int leftButtonPin = 3;
const int rightButtonPin = 4;

#define RF_BROADCASTID 0

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

void loop()
{
  int leftState = 0;
  int rightState = 0;
  
  leftState = digitalRead(leftButtonPin);
  rightState = digitalRead(rightButtonPin);
  
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
   
  int sendLR = 0; // -2 through +2 to send fast-left, normal-left, no-LR, normal-right, or fast-right
  int sendUD = 0; // same for up-through-down
   
  if (lrAnalog <= 175) {
    // full left
    sendLR = -2;
  } else if (lrAnalog <= 347) {
     // Somewhat left
     sendLR = -1;
   } else if (lrAnalog >= 810) {
     // full right
     sendLR = 2;
   } else if (lrAnalog >= 710) {
     // somewhat right
     sendLR = 1;
   }
   
   if (udAnalog <= 160) {
     // full up
     sendUD = 2;
   } else if (udAnalog <= 300) {
    // somewhat up
    sendUD = 1;
   } else if (udAnalog >= 790) {
    // full down
    sendUD = -2;
   } else if (udAnalog >= 640) {
    // somewhat down
    sendUD = -1;
   }   
   
   if (sendLR < 0) {
     rf_send("L", 1);
   }
   if (sendLR > 0) {
     rf_send("R", 1);
   }
   
   if (sendUD < 0) {
     rf_send("B", 1);
   }
   if (sendUD > 0) {
     rf_send("F", 1);
   }
   
   // leftState and rightState are to swing around the head
   if (leftState && rightState) {
     // ignore double-presses.
     leftState = rightState = 0;
   }
   
   if (leftState) {
     rf_send("(", 1);
   } else if (rightState) {
     rf_send(")", 1);
   }
   
   if (sendLR != 0 || sendUD != 0 || leftState || rightState ) {
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


