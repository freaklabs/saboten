#pragma once

/************************************************************************/
// 
//
//
/************************************************************************/
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <avr/pgmspace.h>
#include <limits.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "ds3231.h"

#if defined(__AVR_ATmega1284P__)
    #include <SdFat.h>
#endif

// for alarm
enum
{
    EVERY_SECOND,
    EVERY_MINUTE,
    EVERY_HOUR     ,
    EVERY_DAY    ,
    EVERY_WEEK    ,
    EVERY_MONTH    
};

enum
{
    NO_ALARM,
    ALARM1,
    ALARM2
};

class Saboten 
{
public:
    // inputs
    const uint8_t pinVsolSense     = 28;
    const uint8_t pinVbatSense     = 31;
    const uint8_t pinCdet          = 18;

    // outputs
    const uint8_t pinRtcIntp       = 6;
    const uint8_t pinSdSeln        = 19;

    boolean sleeping = false;
    char buf[50];

    Saboten();
    boolean begin();
    uint8_t getIntp();
    static void rtcIntp();
    boolean rtcIntpRcvd();

    void sleepMcu();
    uint32_t elapsedTime(uint32_t);
    float getVbat();
    float getVsol();

    void rtcSetTime(int yr, int month, int day, int hour, int min, int sec);
    static struct ts rtcGetTime();
    void rtcPrintTime(char *datetime);
    void rtcPrintDate(char *datetime);
    void rtcPrintTimeAndDate(char *datetime);
    void rtcPrintFullTime(char *datetime);
    void rtcSetAlarm(uint8_t alarm, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t repeat);
    void rtcGetAlarm(uint8_t alarm, uint8_t *data);
    uint8_t rtcGetStatus();
    uint8_t rtcGetControl();
    void rtcSetStatus(uint8_t reg);
    void rtcClearAlarm(uint8_t alarm);
    void rtcEnableAlarm(uint8_t alarm);
    void rtcDisableAlarm(uint8_t alarm);
    uint8_t rtcGetTemp();

#if defined(__AVR_ATmega1284P__)
    SdFat sd;
    SdFile file;

    boolean sdBegin(uint8_t csPin);
    boolean sdOpen(const char *filename, uint8_t mode = O_RDWR | O_CREAT | O_APPEND);
    void sdLs();
    boolean sdMkDir(const char *filepath);
    boolean sdExists(const char *filepath);
    boolean sdChDir(const char *filepath);    
    int16_t sdRead();
    boolean sdWrite(const char *data);
    boolean sdClose();
    boolean sdRemove(const char *filename);
    uint32_t sdAvailable();
    static void sdDateTime(uint16_t *date, uint16_t *time);
#endif
};

extern Saboten sab;