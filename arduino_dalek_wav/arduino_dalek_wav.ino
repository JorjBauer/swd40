#include <WaveHC.h>
#include <WaveUtil.h>
#include <SoftwareSerial.h>

/* two control pins */
#define ctl1 A0
#define ctl2 A1

SdReader card;    // This object holds the information for the card
FatVolume vol;    // This holds the information for the partition on the card
FatReader root;   // This holds the information for the volumes root directory
FatReader file;   // This object represent the WAV file 
WaveHC wave;      // This is the only wave (audio) object, since we will only play one at a time

#define CMD_STOP 255

// Number of files.
#define FILE_COUNT 7

char *fileNames[FILE_COUNT] = { "INTER1.WAV", "INTER2.WAV", "INTER3.WAV", "INTER4.WAV", "GUN.WAV", "EXTERM.WAV", "THEME.WAV" };

// index of WAV files in the root directory
uint16_t fileIndex[FILE_COUNT];


SoftwareSerial softSerial(ctl1, ctl2); // RX, TX

/*
 * Define macro to put error messages in flash memory
 */
#define error(msg) error_P(PSTR(msg))

//////////////////////////////////// SETUP
void setup() {
  Serial.begin(115200);
  softSerial.begin(9600);

  if (!card.init()) error("card.init");

  // enable optimized read - some cards may timeout
  card.partialBlockRead(true);

  if (!vol.init(card)) error("vol.init");

  if (!root.openRoot(vol)) error("openRoot");

  PgmPrintln("Index files");
  indexFiles();
  
  pinMode(ctl1, INPUT);
  pinMode(ctl2, INPUT);
}

uint8_t readPins()
{
  if (softSerial.available()) {
    uint8_t c = softSerial.read();
    Serial.println(c);
    if (c >= '1' && c <= '9') {
      return c - '1' + 1;
    } else if (c == 0) {
      return CMD_STOP;
    }
  }
  
  return 0;
}

//////////////////////////////////// LOOP
void loop() 
{
  uint8_t cmd = readPins();
  if (cmd && cmd != CMD_STOP) {
    playByIndex(cmd);
  } else if (cmd == CMD_STOP) {
    wave.stop();
  } else {
    delay(100);
  }
}

/////////////////////////////////// HELPERS
/*
 * print error message and halt
 */
void error_P(const char *str) {
  PgmPrint("Error: ");
  SerialPrint_P(str);
  sdErrorCheck();
  while(1);
}
/*
 * print error message and halt if SD I/O error, great for debugging!
 */
void sdErrorCheck(void) {
  if (!card.errorCode()) return;
  PgmPrint("\r\nSD I/O error: ");
  Serial.print(card.errorCode(), HEX);
  PgmPrint(", ");
  Serial.println(card.errorData(), HEX);
  while(1);
}

/*
 * Find files and save file index.  A file's index is is the
 * index of it's directory entry in it's directory file. 
 */
void indexFiles(void) {
  char name[10];
  
  for (uint8_t i=0; i<FILE_COUNT; i++) {
    Serial.println(fileNames[i]);
    strcpy(name, fileNames[i]); // FIXME: strcpy_P? And PSTRs?
    if (!file.open(root,name)) error("open by name");
    fileIndex[i] = root.readPosition()/32-1;
  }
  PgmPrintln("Done indexing");
}

/*
 * Play file by index and print latency in ms
 */
void playByIndex(uint8_t i) {
  // start time
  uint32_t t = millis();
  
  Serial.print("Starting index ");
  Serial.println(i);
  
  if (wave.isplaying)
    wave.stop();
  
  // open by index
  if (!file.open(root, fileIndex[i])) {
    error("open by index");
  }
  
  // create and play Wave
  if (!wave.create(file)) error("wave.create");
  wave.play();
  
  // print time to open file and start play
//  Serial.println(millis() - t);
  
  // stop after PLAY_TIME ms
//  while (wave.isplaying) {
//    delay(100);
//  }
  
//  while((millis() - t) < PLAY_TIME);
//  wave.stop();
  
  // check for play errors
  sdErrorCheck();

  PgmPrintln("Done playing by index");
}

