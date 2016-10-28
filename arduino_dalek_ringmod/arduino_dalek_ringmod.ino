
/* 
 *   i2s1 (microphone) -> multiply1 (1)
 *   waveform1 -> multiply1 (0)
 *     multiply1 -> mixer1 (0)
 *   playSdWav1 (0,1) -> mixer1 (2,3)
 *   mixer1 -> i2s2 (0,1)
 *   
 *   This way, we can enable mixer1 input 0 for the ring modulated live voice input; or 
 *     mixer1 input 2 and/or 3 for playback from the SD card WAV files.
 */
 
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=130,315
AudioInputI2S            i2s1;           //xy=133,203
AudioSynthWaveform       waveform1;      //xy=149,116
AudioEffectMultiply      multiply1;      //xy=406,183
AudioMixer4              mixer1;         //xy=590,244
AudioOutputI2S           i2s2;           //xy=938,273
AudioConnection          patchCord1(playSdWav1, 0, mixer1, 2);
AudioConnection          patchCord2(playSdWav1, 1, mixer1, 3);
AudioConnection          patchCord3(i2s1, 0, multiply1, 1);
AudioConnection          patchCord4(waveform1, 0, multiply1, 0);
AudioConnection          patchCord5(multiply1, 0, mixer1, 0);
AudioConnection          patchCord6(mixer1, 0, i2s2, 0);
AudioConnection          patchCord7(mixer1, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=478,463
// GUItool: end automatically generated code

// Number of files.
#define FILE_COUNT 10

char *fileNames[FILE_COUNT] = { "0.WAV", "1.WAV", "2.WAV", "3.WAV", "4.WAV", "5.WAV", "6.WAV", "7.WAV", "8.WAV", "9.WAV" };


void setup() {
  Serial.begin(9600);

  // Jorj: guessing that ~100 buffers are okay with the recording we're doing?
  AudioMemory(100);

  mixer1.gain(0, 1.0); // enable live voice input
  mixer1.gain(2, 0.0); // disable the wav file channels
  mixer1.gain(3, 0.0);

/* jorj
 *
 * 14.5 destiny of the daleks
 * 25.64 new series
 * 28.57 genesis
 * 30 power
 * 30.3 5 doctors
 * 30.6 evil
 * 32.25 remembrance
 * 42.85 masterplan
 * dalek invasion of earth: 18.75, 29.12, 29.7, 30.6, 34.5
 */
  waveform1.begin(1.0, 32.25, WAVEFORM_SINE);

  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(SD.begin(10))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  sgtl5000_1.enable();
  sgtl5000_1.volume(0.8);
}

void playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);

  if (playSdWav1.isPlaying()) {
    playSdWav1.stop();
  }

  // Start playing the file.  This sketch continues to
  // run while the file plays.
  playSdWav1.play(filename);

  // A brief delay for the library read WAV info
  delay(5);
}

void playByIndex(int i)
{
  if (i >= 0 && i < FILE_COUNT) {
    playFile(fileNames[i]);
  }
}


void loop() {
  if (Serial.available()) {
    uint8_t c = Serial.read();
    Serial.println(c);
    if (c >= '0' && c <= '9') {
      playByIndex(c - '0');
    } else if (c == 0) {
      playSdWav1.stop();
    }
  }
}

