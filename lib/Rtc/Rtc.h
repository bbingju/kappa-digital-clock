// Rtc.h

#ifndef _RTC_h
#define _RTC_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif
#include <Wire.h>

namespace EmbeddedDevices
{

  class Rtc
  {
  public:
#pragma pack(push)
#pragma pack(1)
    struct DateTime {
      volatile uint8_t sec;
      volatile uint8_t min;
      volatile uint8_t hour;
      volatile uint8_t day;
      volatile uint8_t wday;
      volatile uint8_t mon;
      volatile uint8_t year;
    };
#pragma pack(pop)

  private:
    enum class REG {
		    CONTROL1 = 0x00,
		    CONTROL2 = 0x01,
		    CONTROL3 = 0x02,
		    SECONDS = 0x03,
		    MINUTES = 0x04,
		    HOURS = 0x05,
		    DAYS = 0x06,
		    WEEKDAYS = 0x07,
		    MONTHS = 0x08,
		    YEARS = 0x09,
		    SECOND_ALARM = 0x0A,
		    MINUTE_ALARM = 0x0B,
		    HOUR_ALARM = 0x0C,
		    DAY_ALARM = 0x0D,
		    WEEKDAY_ALARM = 0x0E,
		    CLKOUT_CTL = 0x0F,
		    WATCHDG_TIM_CTL = 0x10,
		    WATCHDG_TIM_VAL = 0x11,
		    TIMESTP_CTL = 0x12,
		    SEC_TIMESTP = 0x13,
		    MIN_TIMESTP = 0x14,
		    HOUR_TIMESTP = 0x15,
		    DAY_TIMESTP = 0x16,
		    MON_TIMESTP = 0x17,
		    YEAR_TIMESTP = 0x18,
		    AGING_OFFSET = 0x19,
		    INTERNAL_REG = 0x1A
    };
    enum class REG_VALUE {
			  CONTROL1_STOP_0 = 0x09,
			  CONTROL1_STOP_1 = 0x29,
			  WATCHDG_TIM_CTL_TI_TP_0 = 0x03
    };


    const uint8_t I2C_ADDR{ 0x51 };
    const uint8_t pin_INT;
    volatile bool b_interrupt;
    DateTime dt;

  public:

    explicit Rtc(uint8_t pin_int)
      : pin_INT(pin_int)
      , b_interrupt(false)
    {
    }

    uint8_t second()  { return (read((uint8_t)REG::SECONDS)  & 0x7F); }
    uint8_t minute()  { return (read((uint8_t)REG::MINUTES)  & 0x7F); }
    uint8_t hour()    { return (read((uint8_t)REG::HOURS)    & 0x1F); }
    uint8_t day()     { return (read((uint8_t)REG::DAYS)     & 0x3F); }
    uint8_t weekday() { return (read((uint8_t)REG::WEEKDAYS) & 0x07); }
    uint8_t month()   { return (read((uint8_t)REG::MONTHS)   & 0x1F); }
    uint8_t year()    { return (read((uint8_t)REG::YEARS)    & 0xFF); }

    DateTime dateTime()
    {
      Wire.beginTransmission(I2C_ADDR);
      Wire.write((uint8_t)REG::SECONDS);
      Wire.endTransmission();

      Wire.requestFrom((int)I2C_ADDR, sizeof(DateTime), true);  // blocking read (request 256 bytes)

      for (uint32_t i = 0; i < sizeof(DateTime); i++) {
	while (Wire.available()) {
	  if (i == 0) {
	    dt.sec = bcdToDec(Wire.read() & 0x7F);
	    break;
	  } else if (i == 1) {
	    dt.min = bcdToDec(Wire.read() & 0x7F);
	    break;
	  } else if (i == 2) {
	    dt.hour = bcdToDec(Wire.read() & 0x3F);
	    break;
	  } else if (i == 3) {
	    dt.day = bcdToDec(Wire.read() & 0x3F);
	    break;
	  } else if (i == 4) {
	    dt.wday = bcdToDec(Wire.read() & 0x07);
	    break;
	  } else if (i == 5) {
	    dt.mon = bcdToDec(Wire.read() & 0x1F);
	    break;
	  } else if (i == 6) {
	    dt.year = bcdToDec(Wire.read() & 0xFF);
	    break;
	  }
	}
      }

      return dt;
    }

    void setup()
    {
      pinMode(pin_INT, INPUT); // THIS SHOULD NOT BE "INPUT_PULLUP"
      Wire.begin();

      Serial.print("REG CNTR1: 0x");
      Serial.println(read((uint8_t)REG::CONTROL1), HEX);
      Serial.print("REG CNTR2: 0x");
      Serial.println(read((uint8_t)REG::CONTROL2), HEX);
      Serial.print("REG CNTR3: 0x");
      Serial.println(read((uint8_t)REG::CONTROL3), HEX);

      //writeRegister(REG::WATCHDG_TIM_CTL, (uint8_t)REG_VALUE::WATCHDG_TIM_CTL_TI_TP_0);
      stop();
    }

    void start()
    {
        writeRegister(REG::CONTROL1, (uint8_t)REG_VALUE::CONTROL1_STOP_0);
        resumeInterrupt();
    }

    void stop()
    {
      writeRegister(REG::CONTROL1, (uint8_t)REG_VALUE::CONTROL1_STOP_1);
      resumeInterrupt();
    }

    uint8_t getIntPin() const { return pin_INT; }
    void setInterrupted() { b_interrupt = true; }
    void resumeInterrupt()
    {
      writeRegister(REG::CONTROL2, 0x00);
      b_interrupt = false;
    }
    bool isInterrupted() const { return b_interrupt; }

    int writeRegister(REG reg, uint8_t data)
    {
      Wire.beginTransmission(I2C_ADDR);
      Wire.write((uint8_t)reg);
      Wire.write(data);
      int err = Wire.endTransmission();
      if (err != 0)
	{
	  Serial.print("I2C error : ");
	  Serial.println(err);
	}
      return err;
    }

    uint8_t read(const uint8_t reg)
    {
      Wire.beginTransmission(I2C_ADDR);
      Wire.write((uint8_t)reg);
      Wire.endTransmission();

      Wire.requestFrom((int)I2C_ADDR, 1, true);  // blocking read (request 256 bytes)

      if (Wire.available()) return Wire.read();
      else                  return 0x00;
    }

    uint8_t bcdToDec(uint8_t val)
    {
      return ((val >> 4) * 10) + (val % 16);
    }

    uint8_t decToBcd(uint8_t val)
    {
      return ((val / 10 * 16) + (val % 10));
    }

    int setDateTime(struct DateTime *pdt)
    {
      dt = *pdt;
      writeRegister(REG::YEARS, decToBcd(dt.year) & 0xFF);
      writeRegister(REG::MONTHS, decToBcd(dt.mon) & 0x1F);
      writeRegister(REG::WEEKDAYS, dt.wday & 0x07);
      writeRegister(REG::DAYS, decToBcd(dt.day) & 0x3F);
      writeRegister(REG::HOURS, decToBcd(dt.hour) & 0x3F);
      writeRegister(REG::MINUTES, decToBcd(dt.min) & 0x7F);
      writeRegister(REG::SECONDS, decToBcd(dt.sec) & 0x7F);

      Serial.print("setDateTime - ");
      printDateTime();

      return 0;
    }

    void printDateTime()
    {
      Serial.print(dt.year, DEC); Serial.print('/');
      Serial.print(dt.mon, DEC); Serial.print('/');
      Serial.print(dt.day, DEC); Serial.print(" ("); Serial.print(dt.wday, DEC); Serial.print(") ");

      Serial.print(dt.hour, DEC); Serial.print(':');
      Serial.print(dt.min, DEC); Serial.print(':');
      Serial.println(dt.sec, DEC);
    }
  };

} // namespace EmbeddedDevices

using Rtc = EmbeddedDevices::Rtc;

#endif

