#include <rom/rtc.h>  // retrieve reset code interface.
#include "Protos.h"

#define TTGO

#define ROTATION_STOPPED_uS 2000000


#define GREEN_LED           25 // TTGO GPIO25
#define RPM_INPUT_PIN       34
#define SCOPE_PIN           35 // avoid GPIO13 SD  !!!! AVOID GPIO2 its for SD CARD AND BOOTPGM!!!
#define SDA                 21 // default, but I2C address conflict with oled
#define SCL                 22 // default, but I2C address conflict with oled

volatile bool bTimerRunning;

static QueueHandle_t RPMDataQueue = NULL;

#define HLEN (1<<4)
typedef struct MESSAGE
{
    long hiTime;
    long loTime;
};

long rpmHistory[HLEN] ={0};

int requestedPwrPct;

volatile unsigned int currentRPS;
volatile unsigned int currentRPM;

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
//------------------------------------------------------------

volatile int timerCntISR;    // Triggervolatile int bTimerRunning;

hw_timer_t * hRotationStoppedTimer = NULL;

portMUX_TYPE criticalSection = portMUX_INITIALIZER_UNLOCKED;

boolean bStalled = false;

void IRAM_ATTR irqTimedOut()
{
   MESSAGE ipc;
   ipc.hiTime = -1;
   ipc.loTime = -1;

   memset(rpmHistory, 0, sizeof(rpmHistory));

   portENTER_CRITICAL_ISR(&criticalSection);
   timerCntISR++;

   bTimerRunning = false;  // needs re-init when spinning starts.
   bStalled = true;

   if (RPMDataQueue) xQueueSendToBackFromISR(RPMDataQueue, &ipc, NULL);

   digitalWrite(GREEN_LED, 0);

   portEXIT_CRITICAL_ISR(&criticalSection);
}

//-------------------------------------------------------------

void setupDeadAirTimer()
{

    hRotationStoppedTimer = timerBegin(0, 80, true);  // 80Mhz/80 = 1Mhz tick rate
    timerAttachInterrupt(hRotationStoppedTimer, &irqTimedOut, true);

    // Sets an alarm to sound every X mS
    timerAlarmWrite(hRotationStoppedTimer, ROTATION_STOPPED_uS, false);  // no auto reload

    //timerSetAutoReload(silenceTimer, false);  // one shot?

    // wait til ready.
    timerAlarmEnable(hRotationStoppedTimer);

}
//-------------------------------------------------------------
void rpmTask( void * parameter )
{
   MESSAGE ipc;
   Serial.printf("%s task running\n", __FUNCTION__);
   long avgPeriod;
   int index = 0;

   unsigned char crlf = 0;

   while( RPMDataQueue != 0 )
   {
      // if no response in 2mS then the data burst is finished.
      if ( xQueueReceive( RPMDataQueue, &ipc,  1000 *portTICK_PERIOD_MS ))
      {
          unsigned long speedinUs;
          speedinUs = ipc.loTime + ipc.hiTime;
          rpmHistory[index] = speedinUs;
          index = (++index) & (HLEN-1);

          avgPeriod = 0;
          for (int i = 0; i < HLEN; i++)
          {
            avgPeriod += rpmHistory[i];
          }
          avgPeriod /= HLEN;

          currentRPS  = (unsigned int)(1000000. / (float) speedinUs);       // PRS instant
          currentRPM = (unsigned int)(1000000. * 60. / (float) avgPeriod);  // RPM average

          //Serial.printf("%4d %5d %5d  %5d", requestedPwrPct, ipc.loTime, ipc.hiTime,  currentRPM);  // into RPM
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

//-------------------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(2000);

  //setOLED(); DO NOT ENABLE. IT FREEZES THE CORE.

  Serial.printf("xxxx\n");
  setupPWM();

  Serial.println("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));

  int cpu0Reset = rtc_get_reset_reason(0);
  Serial.println("CPU0 reset reason: ");
  print_reset_reason(cpu0Reset);

  int cpu1Reset = rtc_get_reset_reason(1);
  Serial.println("CPU1 reset reason: ");
  print_reset_reason(cpu1Reset);

  pinMode(GREEN_LED,OUTPUT);
  ScopeBlips(10);

  delay(1000);

  setupWiFi();  // setup the time first. what a mess.

#if !JTAG_PRESENT
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

  setupDeadAirTimer();

  RPMDataQueue = xQueueCreate(200, sizeof(MESSAGE));

  pinMode(RPM_INPUT_PIN, INPUT);


   xTaskCreatePinnedToCore(
                  rpmTask,       /* Function to implement the task */
                  "rpmTask",      /* Name of the task */
                  30000,          /* Stack size in words */
                  (void *)99,     /* Task input parameter */
                  0,              /* Priority of the task */
                  NULL,           /* Task handle. */
                  0);             /* Core where the task should run */

    xTaskCreatePinnedToCore(
                   controlTask,   /* Function to implement the task */
                   "controlTask",  /* Name of the task */
                   30000,          /* Stack size in words */
                   (void *)99,     /* Task input parameter */
                   0,              /* Priority of the task */
                   NULL,           /* Task handle. */
                   0);             /* Core where the task should run */

    delay(3000);

    attachInterrupt(RPM_INPUT_PIN, irq_handler, CHANGE); // duty cycle varies, measure freq

}
static unsigned long lastLoTime = 1;
static unsigned long lastHiTime = 1;

//---------------------------------------
void irq_handler(void)
{
    MESSAGE ipc;
    static unsigned long lastIrqTime;

    long period;
    unsigned long now;
    long diffTime;
    long duty;

    now = micros();
    diffTime = now - lastIrqTime;
    lastIrqTime = now;

    bool lastState = !digitalRead(RPM_INPUT_PIN);

    // duty cycle should be close to 50%, if not its either starting up or coming to a stop.
    if (lastState)
    {
        lastHiTime = diffTime;
    }
    else
    {
        lastLoTime = diffTime;
    }

    period = lastLoTime + lastHiTime;

    ipc.hiTime = lastHiTime;
    ipc.loTime = lastLoTime;

    if (!bTimerRunning)
    {
        //rearm the timer that monitors the fan not turning.

        bTimerRunning = true;
        timerAlarmWrite(hRotationStoppedTimer, ROTATION_STOPPED_uS , false);  // reload threshold, no auto reload
        timerAlarmEnable(hRotationStoppedTimer);
        digitalWrite(GREEN_LED,1);
    }

    // set rotation timer back to zero.
    timerWrite(hRotationStoppedTimer, 0); //set count to zero.

    //if (RPMDataQueue && !bStalled) xQueueSendToBackFromISR(RPMDataQueue, &period, NULL);
    if (RPMDataQueue) xQueueSendToBackFromISR(RPMDataQueue, &ipc, NULL);

    bStalled = false;

}

void loop()
{
   vTaskSuspend(NULL);
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

// Include Wire Library for I2C
#include <Wire.h>

// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>

// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width

#define SERVOMIN  1       // Minimum value
#define SERVOMAX  4090     // Maximum value

// Define servo motor connections (expand as required)
#define SERVO1  15  //Servo Motor 1 on connector 12

#define SPEED 750  //time in ms b/n steps.

void setupPWM() {

  // Serial monitor setup
  Serial.begin(115200);

  // Print to monitor
  Serial.println("PCA9685 Servo Test");

  // Initialize PCA9685
  pca9685.begin();

  // Set PWM Frequency to x Hz
  pca9685.setPWMFreq(1000);

}

//-------------------------------------------------------------
void controlTask( void * parameter )
{
    Serial.printf("%s started\n", __FUNCTION__);
    while (1)
    {
      int pwm1;
      unsigned int lastRPM;

#if 0
      // Move Motor 1 from 0 to 180 degrees
      for (requestedPwrPct = 0; requestedPwrPct <= 100; requestedPwrPct++) {

        // Determine PWM pulse width
        pwm1 = map(requestedPwrPct, 0, 100, SERVOMIN, SERVOMAX);
        // Write to PCA9685
        pca9685.setPWM(SERVO1, 0, pwm1);
        // Print to serial monitor
        Serial.printf("Motor + = %d (%d)\n ", requestedPwrPct, pwm1);
        //Serial.printf("Motor 1 = %d\n ", requestPctPwr);
        delay(SPEED);
      }

      Serial.printf("fast it can go\n");
      delay(10000);
#endif


      // Move Motor 1 from 180 to 0 degrees
      // don't do above 96% it never settles.

      for (requestedPwrPct = 96; requestedPwrPct >= 0; requestedPwrPct--) {

        lastRPM = 0.0;
        unsigned int tries = 0;
        // Determine PWM pulse width
        pwm1 = map(requestedPwrPct, 0, 100, SERVOMIN, SERVOMAX);
        // Write to PCA9685
        pca9685.setPWM(SERVO1, 0, pwm1);
        // Print to serial monitor
        Serial.printf("Motor 1 - %d (h/w = %d)\n ", requestedPwrPct, pwm1);

        delay(SPEED);
        if (currentRPM == 0) continue;

        while (lastRPM != currentRPM)
        {
            tries++;
            lastRPM = currentRPM;
            delay(SPEED);
            //Serial.printf("last=%d current=%d\n", lastRPM, currentRPM);
        }
        Serial.printf("Motor 1 - %d speed = %5d vs %5d tries =%d\n ",
            requestedPwrPct, currentRPM, lastRPM, tries);
      }

      Serial.printf("as slow as it can go\n");
      delay(10000);
    }
}
