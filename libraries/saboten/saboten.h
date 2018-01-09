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
#include <SdFat.h>
#include "rtc3231.h"

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
    SdFat sd;
    SdFile file;
    char buf[50];
    ts_t time;

    Saboten();
    boolean begin();
    uint8_t getIntp();
    static void rtcIntp();
    boolean rtcIntpRcvd();

    void sleepMcu();
    uint32_t elapsedTime(uint32_t);
    float getVbat();
    float getVsol();

    void setTime(int hour, int min, int sec);
    void setDate(int yr, int mon, int day);
    ts_t getTime();
    char *printTime();
    char *printDate();
    char *printFullTime();
    void setAlarm(uint8_t alarm, uint8_t day, uint8_t hour, uint8_t min, uint8_t alarmType);
    char *getAlarm(uint8_t alarm);
    void enableAlarm(uint8_t alarm);
    void disableAlarm(uint8_t alarm);
    void clearAlarm(uint8_t alarm);
    bool isAlarmOn(uint8_t alarm);
    uint8_t getRtcControl();
    uint8_t getRtcStatus();
    void setRtcStatus(uint8_t reg);
    float getRtcTemp();
};

extern Saboten sab;