#include "rtc3231.h"


// single instance of rtc
RTC3231 rtc;

/************************************************************************/
// 
//
//
/************************************************************************/
RTC3231::RTC3231()
{

}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::begin()
{
    Wire.begin();
    setStatus(0); // clear OSF
    setCtrl(DS3231_INTCN);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::setTime(ts_t *ts)
{
    uint8_t time[7] = {ts->sec, ts->min, ts->hour, ts->wday, ts->mday, ts->mon, ts->year_s};
    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(DS3231_TIME_CAL_ADDR);
    for (int i=0; i<7; i++)
    {
        time[i] = decToBcd(time[i]);
        if (i == 5)
        {
            if (ts->year >= 2000)
            {
                uint8_t cent = 1<<7;
                time[6] = ts->year - 2000;
                time[i] |= cent;
            }
        }
        Wire.write(time[i]);
    }
    Wire.endTransmission();
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::getTime(ts_t *ts)
{
    uint8_t time[7];
    uint8_t cent = 0;
    
    initReadFromAddr(DS3231_TIME_CAL_ADDR);
    Wire.requestFrom(DS3231_I2C_ADDR, 7);

    for (int i=0 ; i<7; i++)
    {
        uint8_t val = Wire.read();
        
        if (i==5)
        {
            time[i] = bcdToDec(val & 0x1F);
            cent = val >> 7;
        }
        else
        {
            time[i] = bcdToDec(val);
        }
    }

    ts->sec = time[0];
    ts->min = time[1];
    ts->hour = time[2];
    ts->mday = time[4];
    ts->mon = time[5];
    ts->year = cent ? (time[6] + 2000) : (1900 + time[6]);
    ts->wday = time[3];
    ts->year_s = time[6];
}


/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::writeToAddr(uint8_t addr, uint8_t val)
{
    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(addr);
    Wire.write(val);
    Wire.endTransmission();
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t RTC3231::readFromAddr(uint8_t addr)
{
    uint8_t ret;

    initReadFromAddr(addr);
    Wire.requestFrom(DS3231_I2C_ADDR, 1);
    ret = Wire.read();
    return ret;
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t RTC3231::initReadFromAddr(uint8_t addr)
{
    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(addr);
    Wire.endTransmission();
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::setCtrl(uint8_t val)
{
    writeToAddr(DS3231_CONTROL_ADDR, val);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::setStatus(uint8_t val)
{
    writeToAddr(DS3231_STATUS_ADDR, val);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::setAging(int8_t val)
{
    writeToAddr(DS3231_AGING_OFFSET_ADDR, (uint8_t)val);
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t RTC3231::getStatus()
{
    return readFromAddr(DS3231_STATUS_ADDR);
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t RTC3231::getCtrl()
{
    return readFromAddr(DS3231_CONTROL_ADDR);
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t RTC3231::getAging()
{
    return (int8_t)readFromAddr(DS3231_AGING_OFFSET_ADDR);
}

/************************************************************************/
// 
//
//
/************************************************************************/
float RTC3231::getTemperature()
{
    float ret;
    uint8_t b1, b0;
    int8_t neg;

    initReadFromAddr(DS3231_TEMPERATURE_ADDR);
    Wire.requestFrom(DS3231_I2C_ADDR, 2);
    b1 = Wire.read();
    b0 = Wire.read() >> 6;

    if (b1 & 0x80)
        neg = b1 | ~((1 << 8) - 1);     
    else
        neg = b1;

    ret = 0.25 * b0 + neg;

    return ret;
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::setAlarm1(uint8_t day, uint8_t hour, uint8_t min, uint8_t seconds, uint8_t alarmType)
{

    uint8_t i, time[4] = {seconds, min, hour, day};
    
    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(DS3231_ALARM1_ADDR);
    for (int i=0; i<4; i++)
    {
        time[i] = decToBcd(time[i]);
        time[i] |= ((alarmType >> i) & 0x01) << 7;
        if (alarmType == MATCH_DAY) time[i] |= 1<<6;
        Wire.write(time[i]);
    }
    Wire.endTransmission();
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::getAlarm1(char *buf, uint8_t len)
{
    uint8_t i, alarmType, time[4];
    initReadFromAddr(DS3231_ALARM1_ADDR);
    Wire.requestFrom(DS3231_I2C_ADDR, 4);

    alarmType = 0;

    // parse the alarm regs and store the flags in the alarmType var
    for (i=0; i<4; i++)
    {
        time[i] = Wire.read();
        alarmType |= ((time[i]>>7 & 0x01) << i);

        if (i==3)
        {
            if (time[i] & 0x40)
            {
                alarmType |= 1<<4;
            }
        }
        time[i] &= 0x3F;
        time[i] = bcdToDec(time[i]);
    }

    snprintf(buf, len, "Sec: %02d Min: %02d Hr: %02d Day: %02d Type: %02X\n", time[0], time[1], time[2], time[3], alarmType);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::clearAlarm1()
{
    uint8_t tmp;
    tmp = getStatus() & ~DS3231_A1F;
    setStatus(tmp);
}

/************************************************************************/
// 
//
//
/************************************************************************/
bool RTC3231::isAlarm1On()
{
    return  getStatus() & DS3231_A1F;
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::enableAlarm1()
{
    uint8_t tmp;
    tmp = getCtrl() | DS3231_A1IE;
    setCtrl(tmp);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::disableAlarm1()
{
    uint8_t tmp;
    tmp = getCtrl() & ~DS3231_A1IE;
    setCtrl(tmp);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::setAlarm2(uint8_t day, uint8_t hour, uint8_t min, uint8_t alarmType)
{
    uint8_t i, time[3] = {min, hour, day};

    // shift alarmType by 1 since there are no seconds in alarm 2
    alarmType >>= 1;

    Wire.beginTransmission(DS3231_I2C_ADDR);
    Wire.write(DS3231_ALARM2_ADDR);
    for (int i=0; i<3; i++)
    {
        time[i] = decToBcd(time[i]);
        time[i] |= ((alarmType >> i) & 0x01) << 7;
        if (alarmType & 0x08) time[i] |= 1<<6;
        Wire.write(time[i]);
    }
    Wire.endTransmission();
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::getAlarm2(char *buf, uint8_t len)
{
    uint8_t i, alarmType, time[3];
    initReadFromAddr(DS3231_ALARM2_ADDR);
    Wire.requestFrom(DS3231_I2C_ADDR, 3);

    // parse the alarm regs and store the flags in the alarmType var
    for (i=0; i<3; i++)
    {
        time[i] = Wire.read();
        alarmType |= ((time[i]>>7 & 0x01) << i);

        if (i==2)
        {
            if (time[i] & 0x40)
            {
                alarmType |= 1<<3;
            }
        }
        time[i] &= 0x3F;
        time[i] = bcdToDec(time[i]);
    }

    snprintf(buf, len, "Min: %02d Hr: %02d Day: %02d Type: %02X\n", time[0], time[1], time[2], alarmType);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::clearAlarm2()
{
    uint8_t tmp;
    tmp = getStatus() & ~DS3231_A2F;
    setStatus(tmp);
}

/************************************************************************/
// 
//
//
/************************************************************************/
bool RTC3231::isAlarm2On()
{
    return getStatus() & DS3231_A2F;
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::enableAlarm2()
{
    uint8_t tmp;
    tmp = getCtrl() | DS3231_A2IE;
    setCtrl(tmp);
}

/************************************************************************/
// 
//
//
/************************************************************************/
void RTC3231::disableAlarm2()
{
    uint8_t tmp;
    tmp = getCtrl() & ~DS3231_A2IE;
    setCtrl(tmp);
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t RTC3231::decToBcd(uint8_t val)
{
    return (uint8_t) ((val / 10 * 16) + (val % 10));
}

/************************************************************************/
// 
//
//
/************************************************************************/
uint8_t RTC3231::bcdToDec(uint8_t val)
{
    return (uint8_t) ((val / 16 * 10) + (val % 16));
}
