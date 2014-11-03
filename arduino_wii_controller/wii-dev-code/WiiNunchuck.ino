/*
 * WiiChuckDemo -- 
 *
 * 2008 Tod E. Kurt, http://thingm.com/
 * 2009/09/25 Modified by Micono Utilities
 *
 */

#include <Wire.h>
#include "nunchuck_funcs.h"

int loop_cnt=0;

byte accx,accy,accz,zbut,cbut,joyx,joyy;

void setup()
{
    Serial.begin(19200);
    nunchuck_setpowerpins();
    nunchuck_init(); // send the initilization handshake    
    //Serial.print("WiiChuckDemo ready\n");
}

void loop()
{
    if( loop_cnt > 20 ) { // every 20 msecs get new data
        loop_cnt = 0;

        nunchuck_get_data();

        accx  = nunchuck_accelx(); // ranges from approx 70 - 182
        accy  = nunchuck_accely(); // ranges from approx 65 - 173
        accz  = nunchuck_accelz(); // ranges from approx ? - ?
        zbut = nunchuck_zbutton();
        cbut = nunchuck_cbutton(); 
        joyx = nunchuck_joyx();
        joyy = nunchuck_joyy();
            
        //x accelation
        Serial.print((byte)accx,DEC);
        Serial.print(",");

        //Y accelation
        Serial.print((byte)accy,DEC);
        Serial.print(",");

        //Z accelation
        Serial.print((byte)accz,DEC);
        Serial.print(",");

        //Z button
        Serial.print((byte)zbut,DEC);
        Serial.print(",");

        //C button
        Serial.print((byte)cbut,DEC);
        Serial.print(",");
        
        //Joy stick X
        Serial.print((byte)joyx,DEC);
        Serial.print(",");
        
        //Joy stick Y
        Serial.println((byte)joyy,DEC);//æœ€å¾Œã¯println

    }
    loop_cnt++;
    delay(1);
}

