#include <rom/rtc.h>  // retrieve reset code interface.

// Include Wire Library for I2C
#include <Wire.h>

// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>

// Define servo motor connections (expand as required)
#define SERVO1  15  //Servo Motor 0 on connector 15
#define SERVO2  14  //Servo Motor 1 on connector 14

uint8_t servo[2] = { SERVO1, SERVO2};

// Creat object to represent PCA9685 at default I2C address

#include "Protos.h"

//-----------------------------------------------------------
float heatIndex, dewPoint, comfortRatio;
float temperature;
float humidity;
byte perception;
char *pcPerception = "TBD";
char *pcComfortString = "TBD";
//-----------------------------------------------------------

#define TTGO

#define ROTATION_STOPPED_uS 2000000

#define RPM_FAN1_PIN        34
#define RPM_FAN2_PIN        35

#define SCOPE_PIN           35 // avoid GPIO13 SD  !!!! AVOID GPIO2 its for SD CARD AND BOOTPGM!!!
#define SDA                 21 // default but I2C address conflict with oled
#define SCL                 22 // default but I2C address conflict with oled


#define HLEN (1<<4)
typedef struct MESSAGE
{
    long hiTime;
    long loTime;
};

long rpmHistory[2][HLEN];

int requestedPwrPct[2];

volatile unsigned int currentRPS[2];
volatile unsigned int currentRPM[2];
volatile bool bTimerRunning[2];
static QueueHandle_t RPMDataQueue[2] = {NULL, NULL};

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

//------------------------------------------------------------

volatile int timerCntISR;    // Triggervolatile int bTimerRunning[fanNum];

hw_timer_t * hRotationStoppedTimer[2] = {NULL, NULL};

portMUX_TYPE criticalSection = portMUX_INITIALIZER_UNLOCKED;

boolean bStalled[2];
//-------------------------------------------------------------

void IRAM_ATTR irqTimedOut(uint8_t fanNum)
{
   MESSAGE ipc;
   ipc.hiTime = -1;
   ipc.loTime = -1;

   memset(rpmHistory[fanNum], 0, sizeof(rpmHistory[fanNum]));

   portENTER_CRITICAL_ISR(&criticalSection);
   timerCntISR++;

   bTimerRunning[fanNum] = false;  // needs re-init when spinning starts.
   bStalled[fanNum] = true;

   if (RPMDataQueue[fanNum]) xQueueSendToBackFromISR(RPMDataQueue[fanNum], &ipc, NULL);

   ////digitalWrite(GREEN_LED, 0);

   portEXIT_CRITICAL_ISR(&criticalSection);
}

void IRAM_ATTR irq0TimedOut()
{
    irqTimedOut(0);
}

void IRAM_ATTR irq1TimedOut()
{
    irqTimedOut(1);
}

//-------------------------------------------------------------

void setupDeadAirTimer()
{

    hRotationStoppedTimer[0] = timerBegin(0, 80, true);  // 80Mhz/80 = 1Mhz tick rate
    hRotationStoppedTimer[1] = timerBegin(1, 80, true);  // 80Mhz/80 = 1Mhz tick rate

    timerAttachInterrupt(hRotationStoppedTimer[0], &irq0TimedOut, true);
    timerAttachInterrupt(hRotationStoppedTimer[1], &irq1TimedOut, true);

    // Sets an alarm to sound every X mS
    timerAlarmWrite(hRotationStoppedTimer[0], ROTATION_STOPPED_uS, false);  // no auto reload
    timerAlarmWrite(hRotationStoppedTimer[1], ROTATION_STOPPED_uS, false);  // no auto reload

    //timerSetAutoReload(silenceTimer, false);  // one shot?

    // wait til ready.
    timerAlarmEnable(hRotationStoppedTimer[0]);
    timerAlarmEnable(hRotationStoppedTimer[1]);

}
//-------------------------------------------------------------
void rpmTask( void * parameter )
{

   MESSAGE ipc;
   long avgPeriod;
   int index = 0;
   unsigned char crlf = 0;
   uint32_t fanNum = (uint32_t) parameter;

   Serial.printf("%s task running fan %d\n", __FUNCTION__, fanNum);


   while( RPMDataQueue[fanNum] != 0 )
   {
      // if no response in 2mS then the data burst is finished.
      if ( xQueueReceive( RPMDataQueue[fanNum], &ipc,  1000 *portTICK_PERIOD_MS ))
      {
          unsigned long speedinUs;
          speedinUs = ipc.loTime + ipc.hiTime;
          rpmHistory[fanNum][index] = speedinUs;
          index = (++index) & (HLEN-1);

          avgPeriod = 0;
          for (int i = 0; i < HLEN; i++)
          {
            avgPeriod += rpmHistory[fanNum][i];
          }
          avgPeriod /= HLEN;

          currentRPS[fanNum]  = (unsigned int)(1000000. / (float) speedinUs);       // PRS instant
          currentRPM[fanNum] = (unsigned int)(1000000. * 60. / (float) avgPeriod);  // RPM average

          //Serial.printf("%4d %5d %5d  %5d", requestedPwrPct, ipc.loTime, ipc.hiTime,  currentRPM[0]);  // into RPM
          //Serial.printf("[%5d %5d]\t", average, timeUsPerRevolution);  // RPS
          //crlf++;
          //if (1 || !(crlf % 4)) Serial.println();
      }
      else
      {
          //Serial.printf(".");
      }

      yield();
   }
   vTaskDelete(NULL);
}

//-------------------------------------------------------------
#define SPIN 100

void ScopeBlips(int numBlips)
{
    static volatile uint64_t spin; //keep out of optimization, use voilatile
    while (numBlips--)
    {
        spin = SPIN;
        digitalWrite(SCOPE_PIN, 1);
        while (spin--);
        spin = SPIN;
        digitalWrite(SCOPE_PIN, 0);
        while (spin--);
    }
}
//-------------------------------------------------------------------------
void print_reset_reason(/*RESET_REASON*/ int reason)
{
  Serial.printf("reset value = %d ", reason);
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET");break;          /**<1, Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET");break;               /**<3, Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET");break;             /**<4, Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5, Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET");break;             /**<6, Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9, RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

#define LINE do{Serial.printf("%s:%d\n",__FUNCTION__,__LINE__);delay(100);}while(0);
//-------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(2000);

  //setOLED(); DO NOT ENABLE. IT FREEZES THE CORE.

  Serial.printf("xxxx\n");
  delay(1000);


  int cpu0Reset = rtc_get_reset_reason(0);
  Serial.print("CPU0 reset reason: ");
  print_reset_reason(cpu0Reset);

  int cpu1Reset = rtc_get_reset_reason(1);
  Serial.print("CPU1 reset reason: ");
  print_reset_reason(cpu1Reset);

  LINE
  //setupWiFi();  // setup the time first. what a mess.
  delay(1000);
  setupDHT();
  setupPWM();


#if 0 // !JTAG_PRESENT
  setupFS();    // fs needs time from wifi.

  //----------------------------------------------------------

  // take a quick look and see if the scope pin is grounded.
  // if it is grounded, that means erase the SD CARD.

  pinMode(SCOPE_PIN,INPUT_PULLUP);

  bool scope = digitalRead(SCOPE_PIN);

  Serial.printf("checking GPIO%d = %d, %s erasing SD card\n", SCOPE_PIN, scope, scope ? "NOT":"");

  if (cpu1Reset != 14 || ERASE_SD_ONPOWERUP || !digitalRead(SCOPE_PIN))
  {
    eraseLogFile();
    Serial.printf("erase done. Waiting for GPIO%d to be released\n", SCOPE_PIN);
    while (!digitalRead(SCOPE_PIN)) delay(500);
    delay(2000);
  }

#endif

  //dumpSDlogs();


  // now SCOPE_GPIO is an output for the scope.
  pinMode(SCOPE_PIN,OUTPUT);


  RPMDataQueue[0] = xQueueCreate(20, sizeof(MESSAGE));
  RPMDataQueue[1] = xQueueCreate(20, sizeof(MESSAGE));

  pinMode(RPM_FAN1_PIN, INPUT);
  pinMode(RPM_FAN2_PIN, INPUT);

#if 0
   xTaskCreatePinnedToCore(
                  rpmTask,       /* Function to implement the task */
                  "rpmTask",      /* Name of the task */
                  30000,          /* Stack size in words */
                  (void *)0,      /* Fan number 0 Task */
                  0,              /* Priority of the task */
                  NULL,           /* Task handle. */
                  0);             /* Core where the task should run */
#endif
#if 1
    xTaskCreatePinnedToCore(
                   rpmTask,       /* Function to implement the task */
                   "rpmTask",      /* Name of the task */
                   30000,          /* Stack size in words */
                   (void *)1,      /* Fan number 1 Task */
                   0,              /* Priority of the task */
                   NULL,           /* Task handle. */
                   0);             /* Core where the task should run */


    // Write to PCA9685
#endif

    pca9685.setPWM(servo[0], 0, 0 ); // all fans off
    pca9685.setPWM(servo[1], 0, 0 );

#if 0
    xTaskCreatePinnedToCore(
                   controlTask,   /* Function to implement the task */
                   "controlTask",  /* Name of the task */
                   30000,          /* Stack size in words */
                   (void *)0,      /* fan 0 */
                   0,              /* Priority of the task */
                   NULL,           /* Task handle. */
                   1);             /* Core where the task should run */
#endif

#if 0
    xTaskCreatePinnedToCore(
                   controlTask,   /* Function to implement the task */
                   "controlTask",  /* Name of the task */
                   30000,          /* Stack size in words */
                   (void *)1,      /* fan 1 */
                   0,              /* Priority of the task */
                   NULL,           /* Task handle. */
                   1);             /* Core where the task should run */
#endif


    attachInterrupt(RPM_FAN1_PIN, irq_handler0, CHANGE);
    attachInterrupt(RPM_FAN2_PIN, irq_handler1, CHANGE);
    setupDeadAirTimer();

    Serial.println("setup done");
}

static unsigned long lastLoTime[2] = {1,1};
static unsigned long lastHiTime[2] = {1,1};
static unsigned long lastIrqTime[2];

//---------------------------------------
void irq_handler(unsigned fanNum)
{
    MESSAGE ipc;

    long period;
    unsigned long now;
    long diffTime;
    long duty;

    now = micros();
    diffTime = now - lastIrqTime[fanNum];
    lastIrqTime[fanNum] = now;

    bool lastState = !digitalRead(RPM_FAN1_PIN);

    // duty cycle should be close to 50%, if not its either starting up or coming to a stop.
    if (lastState)
    {
        lastHiTime[fanNum] = diffTime;
    }
    else
    {
        lastLoTime[fanNum] = diffTime;
    }

    period = lastLoTime[fanNum] + lastHiTime[fanNum];

    ipc.hiTime = lastHiTime[fanNum];
    ipc.loTime = lastLoTime[fanNum];

    if (!bTimerRunning[fanNum])
    {
        //rearm the timer that monitors the fan not turning.

        bTimerRunning[fanNum] = true;
        timerAlarmWrite(hRotationStoppedTimer[fanNum], ROTATION_STOPPED_uS , false);  // reload threshold, no auto reload
        timerAlarmEnable(hRotationStoppedTimer[fanNum]);
        ////digitalWrite(GREEN_LED,1);
    }

    // set rotation timer back to zero.
    timerWrite(hRotationStoppedTimer[fanNum], 0); //set count to zero.

    //if (RPMDataQueue[0] && !bStalled[fanNum]) xQueueSendToBackFromISR(RPMDataQueue[0], &period, NULL);
    if (RPMDataQueue[fanNum]) xQueueSendToBackFromISR(RPMDataQueue[fanNum], &ipc, NULL);

    bStalled[fanNum] = false;

}

void irq_handler0(void)
{
    irq_handler(0);
}

void irq_handler1(void)
{
    irq_handler(1);
}

void loop()
{
   delay(6500);
   loopDHT();
   //vTaskSuspend(NULL);
}
//------------------------------------------------------------------------------

/*
  ESP32 PCA9685 Servo Control
  esp32-pca9685.ino
  Driving multiple servo motors with ESP32 and PCA9685 PWM module
  Use I2C Bus

  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width

#define SERVOMIN  1       // Minimum value
#define SERVOMAX  4090     // Maximum value

#define SPEED 750  //time in ms b/n steps.

void setupPWM() {

  // Serial monitor setup
  Serial.begin(115200);

  // Print to monitor
  Serial.printf("%s: init\n",__FUNCTION__);

  // Initialize PCA9685
  pca9685.begin();

  // Set PWM Frequency to x Hz
  pca9685.setPWMFreq(1000);

}

//-------------------------------------------------------------
void controlTask( void * parameter )
{
    uint32_t fanNum = (uint32_t) parameter;
    Serial.printf("%s started fan %d\n", __FUNCTION__, fanNum);

    while (1)
    {
      int hwPWMval;
      unsigned int lastRPM;

      // don't do above 96% it never settles.

      for (requestedPwrPct[fanNum] = 0; requestedPwrPct[fanNum] < 90; requestedPwrPct[fanNum]++) {

        lastRPM = 0.0;
        unsigned int tries = 0;

        // Determine PWM pulse width
        hwPWMval = map(requestedPwrPct[fanNum], 0, 100, SERVOMIN, SERVOMAX);

        // Write to PCA9685
        pca9685.setPWM(servo[fanNum], 0, hwPWMval);

        // Print to serial monitor
        Serial.printf("Motor %d - %d (h/w = %d)\n ", fanNum, requestedPwrPct[fanNum], hwPWMval);

        delay(SPEED);
        if (currentRPM[fanNum] == 0) continue;

        while (lastRPM != currentRPM[fanNum])
        {
            tries++;
            lastRPM = currentRPM[fanNum];
            delay(SPEED);
            //Serial.printf("fan=%d last=%d current=%d\n", fanNum, lastRPM, currentRPM[0]);
        }
        Serial.printf("Motor %d - %d speed = %5d vs %5d tries =%d\n ",
            fanNum, requestedPwrPct[fanNum], currentRPM[fanNum], lastRPM, tries);
      }

      Serial.printf("fan %d as slow as it can go\n", fanNum);
      delay(10000);
    }
}

//--------------------------------------------------------------------------------
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <Ticker.h>


#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

/**************************************************************/
/* Example how to read DHT sensors from an ESP32 using multi- */
/* tasking.                                                   */
/* This example depends on the Ticker library to wake up      */
/* the task every X  seconds                                  */
/**************************************************************/

DHTesp dht;

void DHTtask(void *pvParameters);
bool getTemperature();
void DHTtimerCallback();

/** Task handle for the light value read task */
TaskHandle_t DHTtaskHandle = NULL;

/** Ticker for temperature reading */
Ticker DHTtimer;

/** Comfort profile */
ComfortState cf;

/** Flag if task should run */
bool bLoopingDHT = false;

/** Pin number for DHT11 data pin */
int dhtPin = 13;

//-------------------------------------------------------------
/**
 * initTemp
 * Setup DHT library
 * Setup task and timer for repeated measurement
 * @return bool
 *    true if task and timer are started
 *    false if task or timer couldn't be started
 */
bool initDHT()
{
  byte resultValue = 0;
  // Initialize temperature sensor
  Serial.printf("%s init\n", __FUNCTION__);

  dht.setup(dhtPin, DHTesp::DHT11);

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      DHTtask,                       /* Function to implement the task */
      "DHTtask ",                     /* Name of the task */
      4000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &DHTtaskHandle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (DHTtaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    // Start update of environment data every 2 seconds
    DHTtimer.attach(10, DHTtimerCallback);
  }
  return true;
}

//-------------------------------------------------------------
void DHTtimerCallback() {
  if (DHTtaskHandle != NULL)
  {
     xTaskResumeFromISR(DHTtaskHandle);
  }
}

/**
 * Task to reads temperature from DHT11 sensor
 * @param pvParameters
 *    pointer to task parameters
 */
void DHTtask(void *pvParameters)
{
  Serial.printf("%s started\n",__FUNCTION__);

  while (1) // tempTask loop
  {
    if (bLoopingDHT)
    {
      // Get temperature values
      getTemperature();
    }
    // Go sleep again
    vTaskSuspend(NULL);
  }
}
//-------------------------------------------------------------

/**
 * getTemperature
 * Reads temperature from DHT11 sensor
 * @return bool
 *    true if temperature could be aquired
 *    false if aquisition failed
*/
bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();

  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0)
  {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
    return false;
  }

  temperature   = newValues.temperature;
  humidity      = newValues.humidity;
  heatIndex     = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  dewPoint      = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  comfortRatio  = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);
  perception = dht.computePerception(temperature, humidity, false);

  switch (perception)
  {
    case Perception_Dry :
        pcPerception = "Dry";
        break;
    case Perception_VeryComfy :
        pcPerception = "VeryComfy";
        break;
    case Perception_Comfy :
        pcPerception = "Comfy";
        break;
    case Perception_Ok :
        pcPerception = "Ok";
        break;
    case Perception_UnComfy :
        pcPerception = "UnComfy";
        break;
    case Perception_QuiteUnComfy :
        pcPerception = "QuiteUnComfy";
        break;
    case Perception_VeryUnComfy :
        pcPerception = "VeryUnComfy";
        break;
    case Perception_SevereUncomfy :
        pcPerception = "SevereUncomfy";
        break;
  }


  switch(cf) {
    case Comfort_OK:
      pcComfortString = "Comfort_OK";
      break;
    case Comfort_TooHot:
      pcComfortString = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      pcComfortString = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      pcComfortString = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      pcComfortString = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      pcComfortString = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      pcComfortString = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      pcComfortString = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      pcComfortString = "Comfort_ColdAndDry";
      break;
    default:
      pcComfortString = "Unknown:";
      break;
  };

  Serial.printf(" Real Temp : %4.1f Humidity : %4.1f Feels Like: %4.1f Dew Point: %4.1f  %s (%s)\n",
      temperature, humidity, heatIndex, dewPoint, pcComfortString, pcPerception);

//  Serial.println(" Real Temp:" + String(newValues.temperature) +
//                  " Humidity :" + String(newValues.humidity) +
//                  "\n Feels Like:" + String(heatIndex) + " Dew Point:" + String(dewPoint) +
//                  " " +
//                  comfortStatus);

  return true;
}

//-------------------------------------------------------------
void setupDHT()
{
  initDHT();

  // Signal end of setup() to tasks
  bLoopingDHT = true;
  LINE;
}
//-------------------------------------------------------------
void loopDHT() {
  if (!bLoopingDHT)
  {
    // Wait 2 seconds to let system settle down
    delay(2000);
    // Enable task that will read values from the DHT sensor
    bLoopingDHT = true;

    if (DHTtaskHandle != NULL)
    {
      vTaskResume(DHTtaskHandle);
    }
  }
  yield();
}



#if 0
//------------------------------------------------------------
// For a connection via I2C using the Arduino Wire include:
#include <Wire.h>              // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
#include "boards.h"

uint8_t vertPositionInPixels = 0;
uint8_t horzPositionInPixels = 0;

// no reset on ttgo for lcd
SSD1306Wire oled(0x3c, I2C_SDA, I2C_SCL);
//--------------------------------------------

#include "font.h"
//static const uint8_t *charSet = Roboto_Mono_14; //Nimbus_Mono_L_Regular_16; // Roboto_Mono_48/32/14/_10;
static const uint8_t *charSet = DejaVu_Sans_Mono_10 ; // Roboto_Mono_48/32/14/_10;

int oprintf(uint8_t row, const char * format,...)
{
    int ret = 0;
    char buffer[120];

    va_list args;
    va_start (args, format);

    uint8_t charHeightPixels = pgm_read_byte(charSet + HEIGHT_POS);
    uint8_t charWidthPixels = pgm_read_byte(charSet + WIDTH_POS);

    // mod true physical to align with a char boundary
    const uint16_t displayWidthInPixels = (oled.width() /charWidthPixels) * charWidthPixels ;
    uint16_t displayHeightInPixels = (oled.height() / charHeightPixels) * charHeightPixels;

    uint16_t maxNumHChars = max(displayWidthInPixels/charWidthPixels, 1);
    //Serial.printf("max no chars = %d\n", maxNumHChars);

    uint8_t maxRowNum = displayHeightInPixels/charHeightPixels - 1;

    //Serial.printf("width adj=%d  real=%d\n", displayWidthInPixels, oled.width());
    //Serial.printf("width adj=%d  real=%d\n", displayHeightInPixels, oled.height());

    if (row <= maxRowNum)
    {
        vertPositionInPixels = row * charHeightPixels;

        ret = vsnprintf (buffer, sizeof(buffer)-1,format, args);

        // erase old text
        oled.setColor(BLACK);
        oled.fillRect(0, vertPositionInPixels, displayWidthInPixels, charHeightPixels);

        oled.setColor(WHITE);

        buffer[min(ret, maxNumHChars-1)] = 0;  // truncate string if too long for font and display

        oled.drawString(0, vertPositionInPixels, buffer);

        //for debug. Write a boarder around the writable text region.
        //oled.drawRect(0, 0, displayWidthInPixels, displayHeightInPixels); // logical phys disp.
        //oled.drawRect(0, 0, oled.width(), oled.height()); // max phys disp.

        oled.display();

        vertPositionInPixels += charHeightPixels;
        if (vertPositionInPixels + charHeightPixels > oled.height()) vertPositionInPixels = 0; //zero based hence >=
    }
    else
    {
      Serial.printf("%s, row out of bounds. Range 0..%d\n", __FUNCTION__, maxRowNum);
    }


    return ret;
}

//---------------------------------------
void quickLog(const char * format,...)
{
    va_list args;
    va_start (args, format);

    int ret;
    char buffer[120];
    char output[150];

    time_t epoch;
    time(&epoch);
    struct tm ts_now;

    ts_now = *localtime(&epoch);

    unsigned long now = micros();

    char  tbuffer[20];
    strftime(tbuffer, sizeof(tbuffer), "%m-%d %H:%M", &ts_now);

    char cSummary = 'X';  // general message

    ret = vsnprintf (buffer,sizeof(buffer)-1,format, args);
    sprintf(output, "%c %lu  %s | %s \n", cSummary, epoch, tbuffer, buffer);

    log2SD(output);
    Serial.println(output);

    va_end (args);

}


//---------------------------------------
char one2oneMapping (const unsigned char foo)
{
    return foo; // default UTF-8 mapper kills all chars > 127 :P
}
//---------------------------------------

int setOLED(void)
{
    oled.init();
    oled.clear();
    oled.flipScreenVertically();

    oled.setFontTableLookupFunction(one2oneMapping);  // now chars above 127 will be printed!!!!!!

    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.setFont(charSet);
//    oprintf(0,"Built: %s", __DATE__);
//    oprintf(1,"Wifi is:%s", MY_SSID);
//    oprintf(2,"SD log=%s JTAG=%s", SDCARD_LOGGING ? "ON":"OFF", SDCARD_LOGGING ? "OFF":"ON");

#if 0
    // test extended character set (now chars at 0x80-0x9F)
    for (int i = 0xA0; i < 0x100; i++)
    {
        oprintf(1,"%02X=%c", i, i);
        oled.display();
        delay(500);
    }
#endif

}

#endif

