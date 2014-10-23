#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/pgmspace.h>

#include "DualMC33926MotorShield.h"

#define M1DIR 7
#define M2DIR 8
#define M1PWM 5 // 9
#define M1FB A0
#define M2DIR 8
#define M2PWM 6 // 10
#define M2FB A1
#define D2 4
#define SF 2 // 12
DualMC33926MotorShield md(M1DIR, M1PWM, M1FB, M2DIR, M2PWM, M2FB, D2, SF);

#define shoulderLeftPin A4
#define shoulderRightPin A5

/* Command List:

L: turn left (with a little forward)
R: turn right (with a little forward)
l: hard left ('r' motor in full reverse)
r: hard right
F: move forward
B: back up
*/


#define FLOAT_TIME 100

#define RF_NODEID 2
#define RF_GROUPID 212

// Speeds are a max of 400, min of -400
#define TURN_SPEED 350
#define FWD_SPEED 350
#define REV_SPEED 350

unsigned long LeftTimer = 0;
unsigned long RightTimer = 0;
unsigned long shoulderLeftTimer = 0;
unsigned long shoulderRightTimer = 0;

void StreamPrint_progmem(Print &out,PGM_P format,...)
{
  // program memory version of printf - copy of format string and result share a buffer                
  // so as to avoid too much memory use                                                                
  char formatString[128], *ptr;
  strncpy_P( formatString, format, sizeof(formatString) ); // copy in from program mem                 
  // null terminate - leave last char since we might need it in worst case for result's \0             
  formatString[ sizeof(formatString)-2 ]='\0';
  ptr=&formatString[ strlen(formatString)+1 ]; // our result buffer...                                 
  va_list args;
  va_start (args,format);
  vsnprintf(ptr, sizeof(formatString)-1-strlen(formatString), formatString, args );
  va_end (args);
  formatString[ sizeof(formatString)-1 ]='\0';
  out.print(ptr);
}

#define Serialprint(format, ...) StreamPrint_progmem(Serial,PSTR(format),##__VA_ARGS__)


void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing motor shield...");
  md.init();
  Serial.println("Initializing RFM12B");
  rf12_initialize(RF_NODEID, RF12_433MHZ, RF_GROUPID);
  Serial.println("Initialized.");
  
  pinMode(shoulderLeftPin, OUTPUT);
  pinMode(shoulderRightPin, OUTPUT);
}

void SetTimer(unsigned long *t)
{
  *t = millis() + FLOAT_TIME;
}


void loop()
{
  if (LeftTimer && LeftTimer < millis()) {
    LeftTimer = 0;
    md.setM1Speed(0);
  }
  if (RightTimer && RightTimer < millis()) {
    RightTimer = 0;
    md.setM2Speed(0);
  }
  
  if (shoulderLeftTimer && shoulderLeftTimer < millis()) {
    shoulderLeftTimer = 0;
    digitalWrite(shoulderLeftPin, LOW);
  }
  if (shoulderRightTimer && shoulderRightTimer < millis()) {
    shoulderRightTimer = 0;
    digitalWrite(shoulderRightPin, LOW);
  }
  
  if (rf12_recvDone()) {
    if (rf12_crc == 0) {
      // CRC==0 means "no errors"

#if 0
      Serial.print("Data received: ");
      for (byte i=0; i<rf12_len; i++) {
        Serialprint("%.2X ", rf12_data[i]);
      }
      Serial.println("");
#endif

      switch (rf12_data[0]) {
        case 'L':
          SetTimer(&LeftTimer);
          md.setM1Speed(TURN_SPEED);
          break;
        case 'R':
          SetTimer(&RightTimer);
          md.setM2Speed(TURN_SPEED);
          break;
        case 'l':
          SetTimer(&LeftTimer);
          SetTimer(&RightTimer);
          md.setSpeeds(TURN_SPEED, -TURN_SPEED);
          break;
        case 'r':
          SetTimer(&LeftTimer);
          SetTimer(&RightTimer);
          md.setSpeeds(-TURN_SPEED, TURN_SPEED);
          break;          
        case 'F':
          SetTimer(&LeftTimer);
          SetTimer(&RightTimer);
          md.setSpeeds(FWD_SPEED, FWD_SPEED);
          break;
        case 'B':
          SetTimer(&LeftTimer);
          SetTimer(&RightTimer);
          md.setSpeeds(-REV_SPEED, -REV_SPEED);
          break;
        case '(':
          SetTimer(&shoulderLeftTimer);
          digitalWrite(shoulderRightPin, LOW);
          digitalWrite(shoulderLeftPin, HIGH);
          break;
        case ')':
          SetTimer(&shoulderRightTimer);
          digitalWrite(shoulderLeftPin, LOW);
          digitalWrite(shoulderRightPin, HIGH);
          break;
      }
    }
  }
}


