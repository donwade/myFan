#include <time.h>
#include <Udp.h>
#include <WiFi.h>
#include "Configure.h"

//---------------------------------------
bool isNTPsyncDone(int lingerTime)
{
	tm timeinfo;
	time_t now;
    char time_output[30];

    uint32_t start = millis();

    // wait for NTP to percolate into local time.
    do {
      time(&now);
      localtime_r(&now, &timeinfo);
      Serial.print(".");
      delay(100);
    } while (((millis() - start) <= (1000 * lingerTime)) && (timeinfo.tm_year < (2016 - 1900)));

    if (timeinfo.tm_year <= (2016 - 1900)) return false;  // the NTP call was not successful

    strftime(time_output, 30, "%a  %d-%m-%y %T", localtime(&now));
    Serial.printf("%s\n", time_output);
    return true;
}

//---------------------------------------
void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}
//---------------------------------------
void setupWiFi()
{
#if WIFI_AVAILABLE

  unsigned int snore = 20;

  //connect to WiFi
  Serial.printf("Connecting to %s ", MY_SSID);
  WiFi.begin(MY_SSID, MY_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if (!snore--) ESP.restart();
  }

  Serial.printf("\n\nCONNECTED to %s\n\ngetting NTP from %s\n\n ", MY_SSID, ntpServer);

  configTime(0, 0, ntpServer);
  setenv("TZ", TZ_INFO, 1);

  if (!isNTPsyncDone(10))  // 10 seconds to find the time.
  {
  	ESP.restart();
  }

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#else
  Serial.printf("%s: Configured for no wifi\n", __FUNCTION__);
  delay(2000);
#endif

  printLocalTime();
}

