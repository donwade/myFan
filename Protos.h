#ifndef __OREGONV2__
#define __OREGONV2__

#include "Configure.h"
#include "TypeDef.h"
#include "Module.h"

#define FULL_DEBUG 0

void dumpGuinessRecords();
void eraseOregon(void); // clear SD files
int log2SD ( const char * format, ... );
void setupWiFi();
void setupFS();
int oprintf(const char * format,...);
int oprintf(uint8_t row, const char * format,...);


#define LINE Serial.printf("%s:%d \n", __FUNCTION__, __LINE__);

#endif // __OREGONV2__

