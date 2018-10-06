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
    volatile rtcFlag = false;
}

/************************************************************************/
// 
//
//
/************************************************************************/
boolean Saboten::begin()
{
    uint8_t reg;

    pinMode(pinSdSeln, INPUT_PULLUP);

    rtc.begin();
    attachInterrupt(RTC_INTP, getIntp, FALLING);

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
        uint8_t alm;
        uint8_t status = rtcGetStatus();

        status &= (DS3231_A2F | DS3231_A1F);

        if (status & DS3231_A1F)
        {
            alm = MINUTE_ALARM;
            status &= ~DS3231_A1F;
        }
        else if (status & DS3231_A2F)
        {
            alm = HOUR_ALARM;
            status &= ~DS3231_A2F;
        }

        // if we get two interrupts, we don't clear the rtcFlag until all interrupts
        // have been serviced.
        if (status == 0)
        {
            rtcFlag = false;
        }

        rtcClearAlarm(alm);
        rtcEnableAlarm(alm);
            return alm;
    }
    else
    {
        return 0;
    }
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
    UCSR0B = 0x00;

#if defined(__AVR_ATmega1284P__)
    uart1Reg = UCSR1B;
    UCSR1B = 0x00;
#endif

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
#if defined(__AVR_ATmega1284P__)
    UCSR1B = uart1Reg;
#endif
    
    UCSR0B = uart0Reg;
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
    volts = (float)(bat * AREF/1023.0);
    return (volts * 2.0);
}   

/**************************************************************************/
// 
/**************************************************************************/
float Saboten::getVsol()
{
    uint16_t sol;
    float volts;

    sol = analogRead(pinVsolSense);
    volts = (float)(sol*AREF/1023.0);
    return (volts * 2.0);
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
void Saboten::rtcSetTime(int yr, int mon, int day, int hour, int min, int sec)
{
    struct ts time;
    memset(&time, 0, sizeof(time));

    time.year = yr;
    time.mon = mon;
    time.mday = day;
    time.hour = hour;
    time.min = min;
    time.sec = sec;
    DS3231_set(time);
}

/**************************************************************************/
// 
/**************************************************************************/
struct ts Saboten::rtcGetTime()
{
    struct ts time;
    DS3231_get(&time);
    return time;
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcPrintTime(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%02d:%02d:%02d", time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcPrintDate(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%04d/%02d/%02d", time.year, time.mon, time.mday);
  memcpy(datetime, tmp, strlen(tmp)+1);
}


/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcPrintTimeAndDate(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%04d/%02d/%02d,%02d:%02d:%02d", time.year, time.mon, time.mday, time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcPrintFullTime(char *datetime)
{
  struct ts time = rtcGetTime();
  memset(tmp, 0, sizeof(tmp));
  sprintf(tmp, "%02d/%02d/%02d %02d:%02d:%02d", time.year-2000, time.mon, time.mday, time.hour, time.min, time.sec);
  memcpy(datetime, tmp, strlen(tmp)+1);
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
        case MINUTE_ALARM:
            DS3231_clear_a1f();
            DS3231_set_a1(sec, min, hour, day, flags); 
        break;

        case HOUR_ALARM:
            DS3231_clear_a2f();
            DS3231_set_a2(min, hour, day, flags); 
        break;

        default:
        return;
    }

    rtcEnableAlarm(alarm);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcGetAlarm(uint8_t alarm, uint8_t *data)
{
    uint8_t addr;
    switch (alarm)
    {
        case MINUTE_ALARM:
            addr = DS3231_ALARM1_ADDR;
        break;

        case HOUR_ALARM:
            addr = DS3231_ALARM2_ADDR;
        break;

        default:
        return;
    }

    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(addr);
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDR, 5);

    for (int i = 0; i < 5; i++) 
    {
        data[i] = Wire.read();
    }
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten::rtcGetControl()
{
    return DS3231_get_creg();
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten::rtcGetStatus()
{
    return DS3231_get_sreg();
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcSetStatus(uint8_t reg)
{
    DS3231_set_sreg(reg);
    return;
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcClearAlarm(uint8_t alarm)
{
    switch (alarm)
    {
        case MINUTE_ALARM:
            DS3231_clear_a1f();
        break;

        case HOUR_ALARM:
            DS3231_clear_a2f();
        break;

        default:
        return;
    }
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcEnableAlarm(uint8_t alarm)
{
    uint8_t reg;
    switch (alarm)
    {
        case MINUTE_ALARM:
            reg = DS3231_get_creg();
            DS3231_set_creg(reg | DS3231_INTCN | DS3231_A1IE);
        break;

        case HOUR_ALARM:
            reg = DS3231_get_creg();
            DS3231_set_creg(reg | DS3231_INTCN | DS3231_A2IE);
        break;
    
        default:
        return;
    }
}


/**************************************************************************/
// 
/**************************************************************************/
void Saboten::rtcDisableAlarm(uint8_t alarm)
{
    uint8_t reg = DS3231_get_addr(DS3231_INTCN);

    switch (alarm)
    {
        case MINUTE_ALARM:
            reg &= ~DS3231_A1IE;
        break;

        case HOUR_ALARM:
            reg &= ~DS3231_A2IE;
        break;

        default:
        return;
    }
    DS3231_set_addr(DS3231_INTCN, reg);
}

/**************************************************************************/
// 
/**************************************************************************/
uint8_t Saboten::rtcGetTemp()
{
    float temp = DS3231_get_treg();
    return round(temp);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SD related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// only on Saboten Pro Devices
#if defined(__AVR_ATmega1284P__)

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdBegin(uint8_t csPin)
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
boolean Saboten3G::sdOpen(const char *filename, uint8_t mode)
{
    return file.open(filename, mode);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::sdLs()
{
    sd.ls(LS_R);
}

/**************************************************************************/
// 
/**************************************************************************/
int16_t Saboten3G::sdRead()
{
    return file.read();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdWrite(const char *data)
{
    file.print(data);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdClose()
{
    file.close();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdRemove(const char *filename)
{
    sd.remove(filename);
}


/**************************************************************************/
// 
/**************************************************************************/
uint32_t Saboten3G::sdAvailable()
{
    return file.available();
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdMkDir(const char *filepath)
{
    return sd.mkdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdChDir(const char *filepath)
{
    return sd.chdir(filepath);
}

/**************************************************************************/
// 
/**************************************************************************/
boolean Saboten3G::sdExists(const char *filename)
{
    return sd.exists(filename);
}

/**************************************************************************/
// 
/**************************************************************************/
void Saboten3G::sdDateTime(uint16_t *date, uint16_t *time)
{
    struct ts now = rtcGetTime();

    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(now.year, now.mon, now.mday);

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(now.hour, now.min, now.sec);
}
#endif