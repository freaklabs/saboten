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

static volatile boolean rtcFlag = false;

// make single instance of saboten to simplify instantation
Saboten sab;

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
    attachInterrupt(RTC_INTP, Saboten::rtcIntp, FALLING);

    for (int i=0; i<2; i++)
    {
        clearAlarm(i+1);
    }

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
    if (rtcFlag)
    {
        rtcFlag = false;

        // do stuff here
    }    
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
void Saboten::sleepMcu()
{
    uint8_t portCReg, portDReg;
    uint8_t uart0Reg, uart1Reg, ddrCReg, ddrDReg;

    DBG_PRINTLN("Sleeping MCU");
    Serial.flush();

    // disable UARTs
    uart0Reg = UCSR0B;
    uart1Reg = UCSR1B;
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
    UCSR0B = uart0Reg;
    UCSR1B = uart1Reg;
    ADCSRA |= (1 << ADEN); 

    PORTC = portCReg;
    DDRC = ddrCReg;

    PORTD = portDReg;
    DDRD = ddrDReg;
}

/**************************************************************************/
// 
/**************************************************************************/
float Saboten::getVbat()
{
    uint16_t bat = analogRead(pinVbatSense);
    return (float)(bat*2*VCC/1023.0);
}   

/**************************************************************************/
// 
/**************************************************************************/
float Saboten::getVsol()
{
    uint16_t sol = analogRead(pinVsolSense);
    return (float)(sol*2*VCC/1023.0);
}   

/************************************************************************/
// elapsedTime - calculates time elapsed from startTime
// startTime : time to start calculating
/************************************************************************/
uint32_t Saboten::elapsedTime(uint32_t startTime)
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
void Saboten::setTime(int hour, int min, int sec)
{
    rtc.getTime(&time);
    time.hour = hour;
    time.min = min;
    time.sec = sec;
    rtc.setTime(&time);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::setDate(int year, int mon, int day)
{
    rtc.getTime(&time);
    time.year = year;
    time.mon = mon;
    time.mday = day;
    rtc.setTime(&time);
}

/**************************************************************************/
// 
/**************************************************************************/
ts_t Saboten::getTime()
{
    rtc.getTime(&time);
    return time;
}

/**************************************************************************/
// 
/**************************************************************************/
char *Saboten::printTime()
{
    rtc.getTime(&time);
    sprintf(buf, "%02d:%02d:%02d", time.hour, time.min, time.sec);
    return buf;
}

/**************************************************************************/
// 
/**************************************************************************/
char *Saboten::printDate()
{
    rtc.getTime(&time);
    sprintf(buf, "%04d/%02d/%02d", time.year, time.mon, time.mday);
    return buf;
}

/**************************************************************************/
// 
/**************************************************************************/
char *Saboten::printFullTime()
{
    rtc.getTime(&time);
    sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d", time.year-2000, time.mon, time.mday, time.hour, time.min, time.sec);
    return buf;
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::setAlarm(uint8_t alarm, uint8_t day, uint8_t hour, uint8_t min, uint8_t alarmType)
{
    if (alarm == ALARM2)
    {
        rtc.setAlarm2(day, hour, min, alarmType);
    }
    else if (alarm == ALARM1)
    {
        rtc.setAlarm1(day, hour, min, 0, alarmType);
    }
    else
    {
    }
}

/*************************************************************

*************************************************************/
char *Saboten::getAlarm(uint8_t alarm)
{
    if (alarm == ALARM2)
    {
        rtc.getAlarm2(buf, 50);
    }
    else if (alarm == ALARM1)
    {
        rtc.getAlarm1(buf, 50);
    }
    else
    {
        sprintf(buf, "NO ALARM");
    }
    return buf;
}

/*************************************************************

*************************************************************/
void Saboten::enableAlarm(uint8_t alarm)
{
    if (alarm == ALARM2)
    {
        rtc.enableAlarm2();
    }
    else if (alarm == ALARM1)
    {
        rtc.enableAlarm1();
    }
    else
    {
    }
}

/*************************************************************

*************************************************************/
void Saboten::disableAlarm(uint8_t alarm)
{
    if (alarm == ALARM2)
    {
        rtc.disableAlarm2();
    }
    else if (alarm == ALARM1)
    {
        rtc.disableAlarm1();
    }
    else
    {
    }
}

/*************************************************************

*************************************************************/
void Saboten::clearAlarm(uint8_t alarm)
{
    if (alarm == ALARM2)
    {
        rtc.clearAlarm2();
    }
    else if (alarm == ALARM1)
    {
        rtc.clearAlarm1();
    }
    else
    {
    }
}

/*************************************************************

*************************************************************/
bool Saboten::isAlarmOn(uint8_t alarm)
{
    if (alarm == ALARM2)
    {
        return rtc.isAlarm2On();
    }
    else if (alarm == ALARM1)
    {
        return rtc.isAlarm1On();
    }
    else
    {
    }
}


/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten::getRtcControl()
{
    return rtc.getCtrl();
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten::getRtcStatus()
{
    return rtc.getStatus();
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::setRtcStatus(uint8_t reg)
{
    rtc.setStatus(reg);
    return;
}


/**************************************************************************/
// 
/**************************************************************************/
float Saboten::getRtcTemp()
{
    return rtc.getTemperature();
}
