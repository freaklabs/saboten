/************************************************************************/
// 
//
//
/************************************************************************/
#include "saboten.h"

#define DEBUG 1

// allows printing or not printing based on the DEBUG VAR
#if (DEBUG == 1)
  #define DBG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif

#define DEFAULT_TIMEOUT 5000
#define RTC_INTP 2
#define VCC 3.3
#define AREF 2.5
#define CTRLZ 0x1A

volatile boolean rtcFlag = false;

// make single instance of saboten to simplify instantation
Saboten saboten;
static char dateTime[100];

/************************************************************************/
// 
//
//
/************************************************************************/
Saboten::Saboten()
{
}

/************************************************************************/
// 
//
//
/************************************************************************/
boolean Saboten::begin()
{
    uint8_t reg;

    rtc.begin();

    for (int i=0; i<2; i++)
    {
        rtcClearAlarm(i+1);
    }

    // clear oscillator stop flag and disable 32k output
    reg = rtcGetStatus();
    reg &= ~(DS3231_OSF | DS3231_EN32K);
    rtcSetStatus(reg);

    //attachInterrupt(RTC_INTP, Saboten::rtcIntp, FALLING); 

  //  sdBegin(pinSdSeln);
  //  file.dateTimeCallback(sdDateTime);


    DBG_PRINTLN(F("Initialization complete"));
    return true;
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t Saboten::getIntp()
{
    // define the interrupt service routine
    return 0;
}

/********************************************************************/
//
//
//
/********************************************************************/
void Saboten::rtcIntp()
{
  rtcFlag = true;
}

/********************************************************************/
//
//
//
/********************************************************************/
boolean Saboten::rtcIntpRcvd()
{
    return rtcFlag;
}


/**************************************************************************/
// Basic Driver Functionality
/**************************************************************************/

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::drvrSleepMcu()
{
    uint8_t portBReg, portCReg, portDReg;
    uint8_t ddrBReg, ddrCReg, ddrDReg;

    Serial.println("Sleeping MCU");
    Serial.flush();

    // disable UARTs
    UCSR0B = 0x00;
    UCSR1B = 0x00;

    // set all inputs
    portCReg = PORTC;
    ddrCReg = DDRC;
    PORTC = 0x00;
    DDRC = 0x00; 

    portDReg = PORTD;
    ddrDReg = DDRD;
    PORTD = 0x00;
    DDRD = 0x00;

    delay(100);

    ADCSRA &= ~(1 << ADEN);    // Disable ADC

    // write sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();                       // setting up for sleep ...
    sleep_mode();
    // sleeping here

    // waking up here
    UCSR0B = 0x98;
    UCSR1B = 0x98;
    ADCSRA |= (1 << ADEN); 

    PORTC = portCReg;
    DDRC = ddrCReg;

    PORTD = portDReg;
    DDRD = ddrDReg;
}

/**************************************************************************/
// 
/**************************************************************************/
float Saboten::drvrGetVbat()
{
    uint16_t bat = analogRead(pinVbatSense);
    return (float)(bat*2*VCC/1023.0);
}   

/**************************************************************************/
// 
/**************************************************************************/
float Saboten::drvrGetVsol()
{
    uint16_t sol = analogRead(pinVsolSense);
    return (float)(sol*2*VCC/1023.0);
}   

/************************************************************************/
// elapsedTime - calculates time elapsed from startTime
// startTime : time to start calculating
/************************************************************************/
uint32_t Saboten::drvrElapsedTime(uint32_t startTime)
{
  uint32_t stopTime = millis();
  
  if (stopTime >= startTime)
  {
    return stopTime - startTime;
  }
  else
  {
    return (ULONG_MAX - (startTime - stopTime));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RTC related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcSetTime(int yr, int mon, int day, int hour, int min, int sec)
{
    ts_t time;
    memset(&time, 0, sizeof(ts_t));

    time.year = yr;
    time.mon = mon;
    time.mday = day;
    time.hour = hour;
    time.min = min;
    time.sec = sec;
    rtc.setTime(&time);
}

/**************************************************************************/
// 
/**************************************************************************/
ts_t Saboten::rtcGetTime()
{
    ts_t time;
    rtc.getTime(&time);
    return time;
}

/**************************************************************************/
// 
/**************************************************************************/
char *Saboten::rtcPrintTime()
{
    ts_t time;
    rtc.getTime(&time);
    sprintf(dateTime, "%02d:%02d:%02d", time.hour, time.min, time.sec);
    return dateTime;
}

/**************************************************************************/
// 
/**************************************************************************/
char *Saboten::rtcPrintDate()
{
    ts_t time = rtcGetTime();
    sprintf(dateTime, "%04d/%02d/%02d", time.year, time.mon, time.mday);
    return dateTime;
}

/**************************************************************************/
// 
/**************************************************************************/
char *Saboten::rtcPrintTimeAndDate()
{
    ts_t time = rtcGetTime();
    sprintf(dateTime, "%04d/%02d/%02d,%02d:%02d:%02d", time.year, time.mon, time.mday, time.hour, time.min, time.sec);
    return dateTime;
}

/**************************************************************************/
// 
/**************************************************************************/
char *Saboten::rtcPrintFullTime()
{
    ts_t time = rtcGetTime();
    sprintf(dateTime, "%02d/%02d/%02d %02d:%02d:%02d", time.year-2000, time.mon, time.mday, time.hour, time.min, time.sec);
    return dateTime;
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcSetAlarm(uint8_t alarm, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t repeat)
{
    uint8_t i;
    uint8_t flags[5] = {0};

    // flags define what calendar component to be checked against the current time in order
    // to trigger the alarm - see datasheet
    // A1M1 (seconds) (0 to enable, 1 to disable)
    // A1M2 (minutes) (0 to enable, 1 to disable)
    // A1M3 (hour)    (0 to enable, 1 to disable) 
    // A1M4 (day)     (0 to enable, 1 to disable)
    // DY/DT          (dayofweek == 1/dayofmonth == 0)
    switch (repeat)
    {
      case EVERY_SECOND:
      for (i=0; i<4; i++) flags[i] = 1;
      break;

      case EVERY_MINUTE:
      for (i=1; i<4; i++) flags[i] = 1;
      break;

      case EVERY_HOUR:
      for (i=2; i<4; i++) flags[i] = 1;
      break;

      case EVERY_DAY:
      for (i=3; i<4; i++) flags[i] = 1;
      break;

      default:
      memset(flags, 0, sizeof(flags)/sizeof(uint8_t));
      break;
    }

    switch (alarm)
    {
        case ALARM1:
            rtc.clearAlarm1();
            rtc.setAlarm1(sec, min, hour, day, flags); 
            rtc.enableAlarm1();
        break;

        case ALARM2:
            rtc.clearAlarm2();
            rtc.setAlarm2(min, hour, day, flags); 
            rtc.enableAlarm2();
        break;

        default:
        return;
    }
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten::rtcGetControl()
{
    return rtc.getCtrl();
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten::rtcGetStatus()
{
    return rtc.getStatus();
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcSetStatus(uint8_t reg)
{
    rtc.setStatus(reg);
    return;
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcGetAlarm(uint8_t alarm, char *data)
{
    if (alarm == ALARM1) 
    {
        rtc.getAlarm1(data, 5);
    }
    else
    {
        rtc.getAlarm2(data, 5);
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcClearAlarm(uint8_t alarm)
{
    if (alarm == ALARM1) 
    {
        rtc.clearAlarm1();
    }
    else
    {
        rtc.clearAlarm2();
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcEnableAlarm(uint8_t alarm)
{
    if (alarm == ALARM1) 
    {
        rtc.enableAlarm1();
    }
    else
    {
        rtc.enableAlarm2();
    }
}


/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcDisableAlarm(uint8_t alarm)
{
    if (alarm == ALARM1) 
    {
        rtc.disableAlarm1();
    }
    else
    {
        rtc.disableAlarm2();
    }
}

/**************************************************************************/
// 
/**************************************************************************/
float Saboten::rtcGetTemp()
{
    return rtc.getTemperature();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SD related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdBegin(uint8_t csPin)
{
    boolean st = sd.begin(csPin, SPI_HALF_SPEED);
    if (!st)
    {
        DBG_PRINTLN(F("[ERROR] Card failed, or not present"));
        sd.initErrorHalt();
    }
    return st;
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdOpen(const char *filename, uint8_t mode)
{
    return file.open(filename, mode);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::sdLs()
{
    sd.ls(LS_R);
}

/**************************************************************************/
// 
/**************************************************************************/
int16_t Saboten::sdRead()
{
    return file.read();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdWrite(const char *data)
{
    file.print(data);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdClose()
{
    file.close();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdRemove(const char *filename)
{
    sd.remove(filename);
}


/**************************************************************************/
// 
/**************************************************************************/
uint32_t Saboten::sdAvailable()
{
    return file.available();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdMkDir(const char *filepath)
{
    return sd.mkdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdChDir(const char *filepath)
{
    return sd.chdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten::sdExists(const char *filename)
{
    return sd.exists(filename);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::sdDateTime(uint16_t *date, uint16_t *time)
{
    ts_t now = rtcGetTime();

    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(now.year, now.mon, now.mday);

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(now.hour, now.min, now.sec);
}