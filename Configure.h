#ifndef __DEFINES_H__
#define __DEFINES_H__

#define ERASE_SD_ONPOWERUP 0

#define WIFI_AVAILABLE  1
#define MY_SSID "DISABLED"

#if WIFI_AVAILABLE
    #undef  MY_SSID
    #define MY_SSID "ignite wifi"
    #define MY_PASSWORD "DEADBEEF04"
#endif

#define SDCARD_LOGGING 0 //define this to be a one or a zero

#if SDCARD_LOGGING == 1
    #define JTAG_PRESENT 0
#elif SDCARD_LOGGING == 0
    #define JTAG_PRESENT 1
#else
    #error "STOP : define SDCARD_LOGGING to be 1 or 0 at line 16"
#endif

//#define ntpServer  "time.google.com"
#define ntpServer  "time.nist.gov"
//#define ntpServer  "pool.ntp.org"
//#define ntpServer  "ch.pool.ntp.org"

//see https://remotemonitoringsystems.ca/time-zone-abbreviations.php
#define TZ_INFO  "EST5EDT"

// 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

#endif  //__DEFINES_H__

