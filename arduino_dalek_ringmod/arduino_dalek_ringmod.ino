
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

// WAV gain of 0.2 is too much louder than the audio input :(
#define WAV_GAIN 0.06
#define MIC_GAIN 3.0

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=67.4444465637207,220.77777481079102
AudioSynthWaveform       waveform1;      //xy=82.33333206176758,78.22222137451172
AudioPlaySdWav           playSdWav1;     //xy=130,315
AudioEffectBitcrusher    bitcrusher1;    //xy=206.7676887512207,224.5454797744751
AudioEffectMultiply      multiply1;      //xy=344.8888702392578,84.11111259460449
AudioFilterStateVariable filter1;        //xy=433.4343605041504,198.98989486694336
AudioMixer4              mixer1;         //xy=590,244
AudioOutputI2S           i2s2;           //xy=938,273
AudioConnection          patchCord1(i2s1, 1, bitcrusher1, 0);
AudioConnection          patchCord2(waveform1, 0, multiply1, 0);
AudioConnection          patchCord3(playSdWav1, 0, mixer1, 2);
AudioConnection          patchCord4(playSdWav1, 1, mixer1, 3);
AudioConnection          patchCord5(bitcrusher1, 0, multiply1, 1);
AudioConnection          patchCord6(multiply1, 0, filter1, 0);
AudioConnection          patchCord7(multiply1, 0, filter1, 1);
AudioConnection          patchCord8(filter1, 0, mixer1, 0);
AudioConnection          patchCord9(filter1, 1, mixer1, 1);
AudioConnection          patchCord10(mixer1, 0, i2s2, 0);
AudioConnection          patchCord11(mixer1, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=478,463
// GUItool: end automatically generated code



// Number of files.
#define FILE_COUNT 10

char *fileNames[FILE_COUNT] = { "0.WAV", "1.WAV", "2.WAV", "3.WAV", "4.WAV", "5.WAV", "6.WAV", "7.WAV", "8.WAV", "9.WAV" };


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  // Jorj: guessing that ~100 buffers are okay with the recording we're doing?
  AudioMemory(100);

  // Shut off all the mixer channels by default
  mixer1.gain(0, 0.0);
  mixer1.gain(1, 0.0);
  mixer1.gain(2, 0.0);
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
//  waveform1.begin(1.0, 32.25, WAVEFORM_SINE);
  waveform1.begin(WAVEFORM_SINE);
  waveform1.amplitude(1.0);
  waveform1.frequency(32.25);

  bitcrusher1.bits(12);
  filter1.frequency(1800); // This defaults to 1000 Hz. We use the low-pass filter output (output 0 of the filter).

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
  mixer1.gain(0, 0.0);
  mixer1.gain(2, WAV_GAIN);
  mixer1.gain(3, WAV_GAIN);
  
  if (i >= 0 && i < FILE_COUNT) {
    playFile(fileNames[i]);
  }
}

void playLiveInput()
{
  mixer1.gain(0, MIC_GAIN); // enable live voice input
  mixer1.gain(2, 0.0); // disable the wav file channels
  mixer1.gain(3, 0.0);
}

void loop() {
  if (Serial1.available()) {
    uint8_t c = Serial1.read();
    Serial.println(c);
    if (c >= '0' && c <= '9') {
      playByIndex(c - '0');
    } else if (c == 0) {
      playSdWav1.stop();
      mixer1.gain(0, 0.0);
      mixer1.gain(1, 0.0);
      mixer1.gain(2, 0.0);
      mixer1.gain(3, 0.0);
    } else if (c == ':') {
      playLiveInput();
    }
  }
}

