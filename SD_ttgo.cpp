#include "Protos.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
//# from the ESP32 arduino library. Mod for ttgo board

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        //Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

#if !JTAG_PRESENT
SPIClass spiSD(HSPI);
#endif

#define SD_CS 13
#define SDSPEED 5000000

#undef NDEBUG

static bool SD_CARD_OK = false;

void setupFS(){

#if !SDCARD_LOGGING
	Serial.printf("\n%s: setup 4 SD card skipped, JTAG allowed\n");
	delay(3000);
	return;
#else

    pinMode(SD_CS, OUTPUT);
    spiSD.begin(14,02,15,SD_CS);       //TTGO SCK,MISO,MOSI,ss

    if(!SD.begin( SD_CS, spiSD, SDSPEED))
    {
   	    Serial.println("Card Mount Failed");
   	    delay(2000); assert(setupFS == 0);
        return;
     }
     else Serial.println("SD card ready.");


    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
	SD_CARD_OK = true;

#if 0
    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
#endif

    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
#endif
}
//-----------------------------------------

#define LOG_FILE "/Oregon.log"

int fgets(File &fs, char *pArray, uint16_t arraySize){

	uint16_t bytesRead = 0;
	int fret = 1;

    while ( pArray && (bytesRead < arraySize) && fret > 0)
    {
        uint8_t data[1];
        fret = fs.read(data, sizeof(data));
        pArray[bytesRead++] = data[0];
        //Serial.printf("\n[%d] = 0x%0X   fret=%d", bytesRead, data[0], fret);

        if (!fret) break;
        if (data[0] != '\n' ) continue;

		pArray[bytesRead] = 0;  // returning a c string
        return bytesRead;
    }
   	return 0;
}
//-----------------------------------------
void dumpSDlogs()
{
#if !SDCARD_LOGGING
	return;
#endif

	if (!SD_CARD_OK) return;

  	Serial.printf("Reading file: %s\n", LOG_FILE);


   File file = SD.open(LOG_FILE);

   if(!file)
   {
	   Serial.println("Failed to open file for reading");
	   return;
   }

   char data[200];
   int slen;
   do
   {
   	   slen = fgets(file, data, sizeof(data));
   	   if (slen && (data[0] == 'R' || data[0] == 'X') ) Serial.print(data);
   } while (slen);

  file.close();
  Serial.printf("%d done.\n", __FUNCTION__);

//-----------------------------------------
}


int log2SD ( const char * format, ... )
{
  int ret;
  char buffer[300];

  if (!SD_CARD_OK) return 0;

  va_list args;
  va_start (args, format);

  ret = vsnprintf (buffer,sizeof(buffer),format, args);

#if !SDCARD_LOGGING
    Serial.println(buffer);
#else
  appendFile(SD, LOG_FILE, buffer);
#endif

  va_end (args);

  return ret;
}

void eraseLogFile(void)
{
#if SDCARD_LOGGING
	Serial.println("erasing SD card in 5 seconds");
	delay(5000);
	deleteFile( SD, LOG_FILE);
	writeFile( SD , LOG_FILE, "start\n");
	Serial.println("SD card erased");
#else
    Serial.println("JTAG present, SD disabled");
#endif

	delay(2000);
}

