/*
WiiClassic Test Code

This code prints the current controller status to the serial port.
Button pressed calls poll whether the button was pressed since the last update call.
as a result, it will just briefly print the last button pressed once.

Tim Hirzel May 2008

2009/10/4 Modified Micono Utilities. (http://micono.cocolog-nifty.com)

*/


#include "Wire.h"
#include "WiiClassic.h"

WiiClassic wiiClassy = WiiClassic();
int loop_cnt=0;

void setup() {
  Wire.begin();
  Serial.begin(19200);
  nunchuck_setpowerpins();
  wiiClassy.begin();
  wiiClassy.update();
}

void loop() {
  if( loop_cnt > 20 ) { // every 20 msecs get new data
    loop_cnt = 0;
    
    //delay(100); // 1ms is enough to not overload the wii Classic, 100ms seems to ease the serial terminal a little
    wiiClassy.update();
  
    //leftShoulder
    Serial.print((byte)wiiClassy.leftShoulderPressed(),DEC);
    Serial.print(",");
    //rightShoulder
    Serial.print((byte)wiiClassy.rightShoulderPressed(),DEC);
    Serial.print(",");
    
    //lzPressed
    Serial.print((byte)wiiClassy.lzPressed(),DEC);
    Serial.print(",");
    //rzPressed
    Serial.print((byte)wiiClassy.rzPressed(),DEC);
    Serial.print(",");
    
    //leftDPressed
    Serial.print((byte)wiiClassy.leftDPressed(),DEC);
    Serial.print(",");
    //rightDPressed
    Serial.print((byte)wiiClassy.rightDPressed(),DEC);
    Serial.print(",");
    //upDPressed
    Serial.print((byte)wiiClassy.upDPressed(),DEC);
    Serial.print(",");
    //downDPressed
    Serial.print((byte)wiiClassy.downDPressed(),DEC);
    Serial.print(",");
  
    //xPressed
    Serial.print((byte)wiiClassy.xPressed(),DEC);
    Serial.print(",");
    //yPressed
    Serial.print((byte)wiiClassy.yPressed(),DEC);
    Serial.print(",");
    //aPressed
    Serial.print((byte)wiiClassy.aPressed(),DEC);
    Serial.print(",");
    //bPressedssed
    Serial.print((byte)wiiClassy.bPressed(),DEC);
    Serial.print(",");
  
    //leftStickXY
    Serial.print(wiiClassy.leftStickX(),DEC);
    Serial.print(",");
    Serial.print(wiiClassy.leftStickY(),DEC);
    Serial.print(",");
  
    //rightStickXY
    Serial.print(wiiClassy.rightStickX(),DEC);
    Serial.print(",");
    Serial.print(wiiClassy.rightStickY(),DEC);
    Serial.print(",");
  
    //selectPressed
    Serial.print((byte)wiiClassy.selectPressed(),DEC);
    Serial.print(",");
    //homePressed
    Serial.print((byte)wiiClassy.homePressed(),DEC);
    Serial.print(",");
    //startPressed
    Serial.println((byte)wiiClassy.startPressed(),DEC);
    //Serial.print(",");

  }
  loop_cnt++;
  delay(1);
  
}
