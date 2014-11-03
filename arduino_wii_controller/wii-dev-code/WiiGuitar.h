/*
  Wii Guitar functions library - Arduino meets Wii Guitar (sold/used with Guitar Hero 3)
 
  2008 Jason Leyrer - http://www.jleyrer.net
 
  This library is an adaptation of Tod E. Kurt's (http://todbot.com/blog/) Wii Nunchuck 
  functions lib, which uses Wii Nunchuck reading code originally from Windmeadow Labs
  (http://www.windmeadow.com/node/42). 

  Questions/Suggestions? Email me at jleyrer [a] gmail.com
  
*/

#include <Wire.h>
#include <Arduino.h>

static uint8_t guitar_buf[6];   // array to store guitar data,

// initialize the I2C system, join the I2C bus,
// and tell the guitar we're talking to it
static void guitar_init()
{ 
            Wire.beginTransmission(0x52);	// device address
            Wire.write(0xF0);
            Wire.write(0x55);
            Wire.endTransmission();
            delay(1);
            Wire.beginTransmission(0x52);
            Wire.write(0xFB);
            Wire.write(0x01);
            Wire.write(byte(0x00));
            Wire.endTransmission();
            Serial.println("Init complete");
}

// Send a request for data to the guitar
static void guitar_send_request()
{
    Wire.beginTransmission(0x52);// transmit to device 0x52
    Wire.write(byte(0x00));// sends one byte
    Wire.endTransmission();// stop transmitting
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
static char guitar_decode_byte (char x)
{
//    x = (x ^ 0x17) + 0x17;
    return x;
}

// Receive data back from the guitar, 
// returns 1 on successful read. returns 0 on failure
static int guitar_get_data()
{
    int cnt=0;
    Wire.requestFrom (0x52, 6);// request data from nunchuck
    while (Wire.available ()) {
        // receive byte as an integer
        guitar_buf[cnt] = guitar_decode_byte(Wire.read());
        Serial.print(guitar_buf[cnt],HEX);
        cnt++;
    }
    guitar_send_request();  // send request for next data payload
    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5) {
        return 1;   // success
    }
    return 0; //failure
}

// Print the input data we have recieved
static void guitar_print_data()
{ 
    static int i=0;
    int joy_x_axis = guitar_buf[0];
    int joy_y_axis = guitar_buf[1];
    // guitar_buf[2] isn't used, as far as I can tell
    int whammy_bar = guitar_buf[3]; // only least significant four bits are used
    
    // The rest of the bytes are shared by the guitar's buttons
    
    int strum_up = (guitar_buf[5] >> 0) & 1;
    int strum_down = (guitar_buf[4] >> 6) & 1;
    int minus_button = (guitar_buf[4] >> 4) & 1;
    int plus_button = (guitar_buf[4] >> 2) & 1;
    int green_button = (guitar_buf[5] >> 4) & 1;
    int red_button = (guitar_buf[5] >> 6) & 1;
    int yellow_button = (guitar_buf[5] >> 3) & 1;
    int blue_button = (guitar_buf[5] >> 5) & 1;
    int orange_button = (guitar_buf[5] >> 7) & 1;
      
    Serial.print("Joystick: ");
    Serial.print("x=");
    Serial.print(joy_x_axis,DEC);
    Serial.print(" y=");
    Serial.print(joy_y_axis, DEC);
    Serial.print("  |  ");

    Serial.print("Green: ");
    Serial.print(green_button,DEC);
    Serial.print("  ");
    
    Serial.print("Red: ");
    Serial.print(red_button,DEC);
    Serial.print("  ");
    
    Serial.print("Yellow: ");
    Serial.print(yellow_button,DEC);
    Serial.print("  ");
    
    Serial.print("Blue: ");
    Serial.print(blue_button,DEC);
    Serial.print("  ");
    
    Serial.print("Orange: ");
    Serial.print(orange_button,DEC);
    Serial.print("  |  ");
    
    Serial.print("D-Strum: ");
    Serial.print(strum_down,DEC);
    Serial.print("  ");
    
    Serial.print("U-Strum: ");
    Serial.print(strum_up,DEC);
    Serial.print("  |  ");
    
    Serial.print("Whammy Bar: ");
    Serial.print(whammy_bar, DEC);
    Serial.print("  |  ");
    
    Serial.print("(+) Knob: ");
    Serial.print(plus_button,DEC);
    Serial.print("  ");
    
    Serial.print("(-) Knob: ");
    Serial.print(minus_button,DEC);
    
    Serial.print("\r\n");  // newline
    i++;
}

/*
    Colored Fret Buttons
*/

// returns green button state: 1=pressed, 0=notpressed
static int guitar_green_button()
{
    return ((guitar_buf[5] >> 4) & 1) ? 0 : 1;
}

// returns  red button state: 1=pressed, 0=notpressed
static int guitar_red_button()
{
    return ((guitar_buf[5] >> 6) & 1) ? 0 : 1;
}

// returns yellow  button state: 1=pressed, 0=notpressed
static int guitar_yellow_button()
{
    return ((guitar_buf[5] >> 3) & 1) ? 0 : 1;
}

// returns blue button state: 1=pressed, 0=notpressed
static int guitar_blue_button()
{
    return ((guitar_buf[5] >> 5) & 1) ? 0 : 1;
}

// returns orange button state: 1=pressed, 0=notpressed
static int guitar_orange_button()
{
    return ((guitar_buf[5] >> 7) & 1) ? 0 : 1;
}

/*
    Joystick
*/

// returns value of x-axis joystick
static int guitar_joyx()
{
    return guitar_buf[0]; 
}

// returns value of y-axis joystick
static int guitar_joyy()
{
    return guitar_buf[1];
}

/*
  Strum Direction
*/

// returns downward strum state (pushing down on the strum bar): 1=pressed, 0=notpressed
static int guitar_strum_down()
{
    return ((guitar_buf[4] >> 6) & 1) ? 0 : 1;
}
// returns upward strum state (pushing up on the strum bar): 1=pressed, 0=notpressed
static int guitar_strum_up()
{
    return ((guitar_buf[5] >> 0) & 1) ? 0 : 1;
}

/*
    Plus/Minus Knob Buttons
*/

// returns minus (-) button state: 1=pressed, 0=notpressed
static int guitar_minus_button()
{
    return ((guitar_buf[4] >> 4) & 1) ? 0 : 1;
}
// returns plus (+) button state: 1=pressed, 0=notpressed
static int guitar_plus_button()
{
    return ((guitar_buf[4] >> 2) & 1) ? 0 : 1;
}

/*
    Whammy Bar
    
*/

// return whammy bar postion: value ranges from 240 (0xF0) when the bar is at rest
// to 250 (0xFA) when the bar is pressed all the way down.
static int guitar_whammy_bar()
{
  return guitar_buf[3];
}
