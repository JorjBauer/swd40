/*
 * Wii Classic -- Use a Wii Classic Controller
 * by Tim Hirzel http://www.growdown.com
 * 
 * 
 * This code owes thanks to the code by these authors:
 * Tod E. Kurt, http://todbot.com/blog/
 *
 * The Wii Nunchuck reading code is taken from Windmeadow Labs
 * http://www.windmeadow.com/node/42
 * 
 * and the reference document on the wii linux site:
 * http://www.wiili.org/index.php/Wiimote/Extension_Controllers/Classic_Controller
 *
 * 2009/10/04 Modified Micono Utilities.(http://micono.cocolog-nifty.com)
 *
 */

#include <Arduino.h>


#ifndef WiiClassic_h
#define WiiClassic_h

// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
    DDRC |= _BV(pwrpin) | _BV(gndpin);
    PORTC &=~ _BV(gndpin);
    PORTC |=  _BV(pwrpin);
    delay(100);  // wait for things to stabilize        
}

class WiiClassic {    
    private:
        byte cnt;
        uint8_t status[4];		// array to store wiichuck output
        byte averageCounter;
        //int accelArray[3][AVERAGE_N];  // X,Y,Z

        uint8_t buttons[2];
        uint8_t lastButtons[2];


    public:

        void begin() 
        {
            Wire.begin();
            cnt = 0;
            averageCounter = 0;
            Wire.beginTransmission (0x52);	// transmit to device 0x52
            Wire.write (0x40);		// sends memory address
            Wire.write (byte(0x00));		// sends memory address
            Wire.endTransmission ();	// stop transmitting
            lastButtons[0] = 0xff;
            lastButtons[1] = 0xff;
            buttons[0] = 0xff;
            buttons[1] = 0xff;

            update();         

        }


        void update() {

            Wire.requestFrom (0x52, 6);	// request data from nunchuck
            while (Wire.available ()) {
                // receive byte as an integer
                if (cnt < 4) {
                    status[cnt] = _nunchuk_decode_byte (Wire.read()); //_nunchuk_decode_byte () 
                } else {
                    lastButtons[cnt-4] = buttons[cnt-4];
                    buttons[cnt-4] =_nunchuk_decode_byte (Wire.read());
                }
                cnt++;
            }
            if (cnt > 5) {
                _send_zero(); // send the request for next bytes
                cnt = 0;                   
            }
        }

        byte * getRawStatus() {
            return status;
        }

        byte * getRawButtons() {
            return buttons;
        }

        boolean leftShoulderPressed() {
            return _PressedRowBit(0,5);
        }

        boolean rightShoulderPressed() {
            return _PressedRowBit(0,1);
        }

        boolean lzPressed() {
            return _PressedRowBit(1,7);
        }

        boolean rzPressed() {
            return _PressedRowBit(1,2);
        }

        boolean leftDPressed() {
            return _PressedRowBit(1,1);
        }

        boolean rightDPressed() {
            return _PressedRowBit(0,7);
        }

        boolean upDPressed() {
            return _PressedRowBit(1,0);
        }

        boolean downDPressed() {
            return _PressedRowBit(0,6);
        }

        boolean selectPressed() {
            return _PressedRowBit(0,4);
        }

        boolean homePressed() {
            return _PressedRowBit(0,3);
        }

        boolean startPressed() {
            return _PressedRowBit(0,2);
        }

        boolean xPressed() {
            return _PressedRowBit(1,3);
        }

        boolean yPressed() {
            return _PressedRowBit(1,5);
        }

        boolean aPressed() {
            return _PressedRowBit(1,4);
        }

        boolean bPressed() {
            return _PressedRowBit(1,6);
        }

        int rightShouldPressure() {
            return status[3] & 0x1f; //rightmost 5 bits
        }

        int leftShouldPressure() {
            return ((status[2] & 0x60) >> 2) + ((status[3] & 0xe0) >> 5); //rightmost 5 bits
        }

        int leftStickX() {
            return  ( (status[0] & 0x3f) >> 1); //Modified 
        }

        int leftStickY() {
            return  ((status[1] & 0x3f)) >> 1; //Modified 
        }

        int rightStickX() {
            return (((status[0] & 0xc0) >> 3) + ((status[1] & 0xc0) >> 5) +  ((status[2] & 0x80) >> 7));

        }

        int rightStickY() {
            return (status[2] & 0x1f);  	
        }


    private:
        boolean _PressedRowBit(byte row, byte bit) {
            byte mask = (1 << bit);
            return (!(buttons[row] & mask )) && (lastButtons[row] & mask);
        }


        byte _nunchuk_decode_byte (byte x)
        {
            x = (x ^ 0x17) + 0x17;
            return x;
        }

        void _send_zero()
        {
            Wire.beginTransmission (0x52);	// transmit to device 0x52
            Wire.write (byte(0x00));		// sends one byte
            Wire.endTransmission ();	// stop transmitting
        }

};



#endif
