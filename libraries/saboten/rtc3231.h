/************************************************************************/
// 
//
//
/************************************************************************/
#pragma once 
#include <Wire.h>
#include <Arduino.h>

#define DS3231_I2C_ADDR  0x68

// registers
enum
{
    DS3231_TIME_CAL_ADDR        =   0x00,
    DS3231_ALARM1_ADDR          =   0x07,
    DS3231_ALARM2_ADDR          =   0x0B,
    DS3231_CONTROL_ADDR         =   0x0E,
    DS3231_STATUS_ADDR          =   0x0F,
    DS3231_AGING_OFFSET_ADDR    =   0x10,
    DS3231_TEMPERATURE_ADDR     =   0x11
};

// control reg bits
enum
{
    DS3231_A1IE    = 0x01,
    DS3231_A2IE    = 0x02,
    DS3231_INTCN   = 0x04
};

// status reg bits
enum
{
    DS3231_A1F      = 0x01,
    DS3231_A2F      = 0x02,
    DS3231_EN32K    = 0x08,
    DS3231_OSF      = 0x80
};

// alarm masks
enum 
{
    EVERY_SECOND    = 0x0F,
    EVERY_MINUTE    = 0x0F,
    MATCH_SECOND    = 0x0E,    // match seconds
    MATCH_MINUTE    = 0x0C,    // match minutes *and* seconds
    MATCH_HOUR      = 0x08,    // match hours *and* minutes, seconds
    MATCH_DATE      = 0x00,    // match date *and* hours, minutes, seconds
    MATCH_DAY       = 0x10    // match day *and* hours, minutes, seconds
};

typedef struct 
{
    uint8_t sec;         /* seconds */
    uint8_t min;         /* minutes */
    uint8_t hour;        /* hours */
    uint8_t mday;        /* day of the month */
    uint8_t mon;         /* month */
    int16_t year;        /* year */
    uint8_t wday;        /* day of the week */
    uint8_t yday;        /* day in the year */
    uint8_t isdst;       /* daylight saving time */
    uint8_t year_s;      /* year in short notation*/
} ts_t;

class RTC3231
{
public:
    RTC3231();
    void begin();
    void setTime(ts_t *);
    void getTime(ts_t *);

    void writeToAddr(uint8_t, uint8_t);
    uint8_t readFromAddr(uint8_t);
    uint8_t initReadFromAddr(uint8_t addr);

    void setCtrl(uint8_t);
    void setStatus(uint8_t);
    void setAging(int8_t);

    uint8_t getCtrl();
    uint8_t getStatus();
    uint8_t getAging();
    float getTemperature();

    void setAlarm1(uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t alarmType);
    void getAlarm1(char *buf, uint8_t len);
    void clearAlarm1();
    bool isAlarm1On();
    void enableAlarm1();
    void disableAlarm1();

    void setAlarm2(uint8_t day, uint8_t hour, uint8_t min, uint8_t alarmType);
    void getAlarm2(char *buf, uint8_t len);
    void clearAlarm2();
    bool isAlarm2On();
    void enableAlarm2();
    void disableAlarm2();

    uint8_t decToBcd(uint8_t val);
    uint8_t bcdToDec(uint8_t val);

private:

};

extern RTC3231 rtc;