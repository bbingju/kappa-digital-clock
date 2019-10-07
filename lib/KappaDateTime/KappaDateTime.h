#ifndef KAPPADATETIME_H
#define KAPPADATETIME_H

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <VariableTimedAction.h>
#include <Rtc.h>
#include <Adafruit_GPS.h>
#include <time.h>
#include "lunar_cal.h"

class KappaDateTime: public VariableTimedAction {
public:
    KappaDateTime(): _rtc(5), _mode(NORMAL_MODE) {
        _lunardate[0] = 0;
        _lunardate[1] = 1;
        _lunardate[2] = 1;
    }

    ~KappaDateTime() {}

    void init(void (*notify)(void *)) {
        _notify = notify;
        _rtc.setup();
        _rtc.start();
    }

    Rtc::DateTime* current() {
        return _current;
    }

    void saveSettingTime() {
        if (_setting_time_modified)
            setCurrent(&_setting_time);
    }

    void setCurrent(Rtc::DateTime *current) {
        if (current) {
            bool run = isRunning();

            if (run)
                toggleRunning();

            _rtc.stop();
            _rtc.setDateTime(current);
            _rtc.start();

            if (run)
                toggleRunning();
        }
    }

    void setCurrent(Adafruit_GPS *obj) {
        float s = obj->seconds + obj->milliseconds/1000.f + obj->secondsSinceTime();
        int m = obj->minute;
        int h = obj->hour;
        int d = obj->day;
        int mon = obj->month;
        int y = obj->year;

        if (d == 0 && mon == 0 && y == 0) {
            y   = _real_time.year;
            mon = _real_time.mon;
            d   = _real_time.day;
        }

        while (s >= 60) { s -= 60; m++; }
        while (m >= 60) { m -= 60; h++; }
        while (h >= 24) { h -= 24; d++; }
        uint8_t last_day = getLastDayOfMonth(y, mon);
        while (d > last_day) { d -= last_day; mon++; }
        while (mon > 12) { mon -= 12; y++; }

        struct tm user_stime;
        user_stime.tm_year   = 2000 + y - 1900; // 주의: 년도는 1900년부터 시작
        user_stime.tm_mon    = mon - 1; // 주의: 월은 0부터 시작
        user_stime.tm_mday   = d;
        user_stime.tm_hour   = h;
        user_stime.tm_min    = m;
        user_stime.tm_sec    = s;
        user_stime.tm_isdst  = 0; // 썸머 타임 사용 안함

        time_t t = mktime(&user_stime); // UTC
        t += (3600 * 9); /* Asia/Seoul */

        struct tm *new_time = localtime(&t);

        _setting_time.year = new_time->tm_year - 100;
        _setting_time.mon  = new_time->tm_mon + 1;
        _setting_time.day  = new_time->tm_mday;
        _setting_time.wday = new_time->tm_wday;
        _setting_time.hour = new_time->tm_hour;
        _setting_time.min  = new_time->tm_min;
        _setting_time.sec  = new_time->tm_sec;

        setCurrent(&_setting_time);
    }

    void changeToNormalMode() {
        _current = &_real_time;
        _mode = NORMAL_MODE;
    }

    void changeToSettingMode() {
        _setting_time = _real_time;
        _current = &_setting_time;
        _setting_time_modified = false;
        _mode = SETTING_MODE;
    }

    bool isSettingMode() {
        return _mode == SETTING_MODE;
    }

    uint8_t * lunarDate() {
        return _lunardate;
    }

    void printCurrent() {
        Serial.print("Current - ");
        Serial.print(_current->year, DEC); Serial.print('-');
        Serial.print(_current->mon, DEC); Serial.print('-');
        Serial.print(_current->day, DEC); Serial.print(" ("); Serial.print(_current->wday, DEC); Serial.print(')');

        Serial.print(_current->hour, DEC); Serial.print(':');
        Serial.print(_current->min, DEC); Serial.print(':');
        Serial.println(_current->sec, DEC);

        Serial.print(" (");
        Serial.print(_lunardate[0], DEC); Serial.print('-');
        Serial.print(_lunardate[1], DEC); Serial.print('-');
        Serial.print(_lunardate[2], DEC);
        Serial.println(")");
    }

    void increaseYear() {
        if (isSettingMode()) {
            uint8_t y = _setting_time.year;
            _setting_time.year = y = (y >= 50) ? 0 : y + 1;
            _setting_time.wday = getDayOfWeek(y, _setting_time.mon, _setting_time.day);
            getLunarDate(getTotalDaySolar(y, _setting_time.mon, _setting_time.day), _lunardate);
        }
    }

    void increaseMonth() {
        if (isSettingMode()) {
            uint8_t m = _setting_time.mon;
            _setting_time.mon = m = (m >= 12) ? 1 : m + 1;
            uint8_t d = getLastDayOfMonth(_setting_time.year, m);
            if (_setting_time.day > d)
                _setting_time.day = d;
            _setting_time.wday = getDayOfWeek(_setting_time.year, m, _setting_time.day);
            getLunarDate(getTotalDaySolar(_setting_time.year, m, _setting_time.day), _lunardate);
        }
    }

    void increaseDay() {
        if (isSettingMode()) {
            uint8_t d = _setting_time.day;
            _setting_time.day = d = (d >= getLastDayOfMonth(_setting_time.year, _setting_time.mon)) ? 1 : d + 1;
            _setting_time.wday = getDayOfWeek(_setting_time.year, _setting_time.mon, d);
            getLunarDate(getTotalDaySolar(_setting_time.year, _setting_time.mon, d), _lunardate);
        }
    }

    void increaseHour() {
        if (isSettingMode()) {
            uint8_t h = _setting_time.hour;
            _setting_time.hour = h = (h >= 23) ? 0 : h + 1;
        }
    }

    void increaseMinute() {
        if (isSettingMode()) {
            uint8_t m = _setting_time.min;
            _setting_time.min = m = (m >=59) ? 0 : m + 1;

            // This is scenario.
            _setting_time_modified = true;
            _setting_time.sec = 0;
        }
    }

    void decreaseYear() {
        if (isSettingMode()) {
            uint8_t y = _setting_time.year;
            _setting_time.year = y = (y == 0) ? 50 : y - 1;
            _setting_time.wday = getDayOfWeek(y, _setting_time.mon, _setting_time.day);
            getLunarDate(getTotalDaySolar(y, _setting_time.mon, _setting_time.day), _lunardate);
        }
    }

    void decreaseMonth() {
        if (isSettingMode()) {
            uint8_t m = _setting_time.mon;
            _setting_time.mon = m = (m == 1) ? 12 : m - 1;
            uint8_t d = getLastDayOfMonth(_setting_time.year, m);
            if (_setting_time.day > d)
                _setting_time.day = d;
            _setting_time.wday = getDayOfWeek(_setting_time.year, m, _setting_time.day);
            getLunarDate(getTotalDaySolar(_setting_time.year, m, _setting_time.day), _lunardate);
        }
    }

    void decreaseDay() {
        if (isSettingMode()) {
            uint8_t d = _setting_time.day;
            _setting_time.day = d = (d == 1) ? getLastDayOfMonth(_setting_time.year, _setting_time.mon) : d - 1;
            _setting_time.wday = getDayOfWeek(_setting_time.year, _setting_time.mon, d);
            getLunarDate(getTotalDaySolar(_setting_time.year, _setting_time.mon, d), _lunardate);
        }
    }

    void decreaseHour() {
        if (isSettingMode()) {
            uint8_t h = _setting_time.hour;
            _setting_time.hour = (h == 0) ? 23 : h - 1;
        }
    }

    void decreaseMinute() {
        if (isSettingMode()) {
            uint8_t m = _setting_time.min;
            _setting_time.min = (m == 0) ? 59 : m - 1;
            // This is scenario.
            _setting_time_modified = true;
            _setting_time.sec = 0;
        }
    }

private:
    Rtc _rtc;
    Rtc::DateTime _setting_time { 0, 0, 0, 1, 0, 1, 0 };
    Rtc::DateTime _real_time { 0, 0, 0, 1, 0, 1, 0 };
    Rtc::DateTime *_current = &_real_time;
    uint8_t _lunardate[3];
    void (*_notify)(void *);
    bool _setting_time_modified = false;

    enum MODE {
        NORMAL_MODE,
        SETTING_MODE,
    } _mode;

    unsigned long run() {

        if (_mode == NORMAL_MODE) {

            // Sync current time
            Rtc::DateTime time = _rtc.dateTime();
            if (memcmp(&time, _current, sizeof(Rtc::DateTime))) {
                *_current = time;
                if (_current->mon == 0 && _current->day == 0) {
                    _current->year = 10;
                    _current->mon  = 1;
                    _current->day  = 1;
                    setCurrent(_current);
                }
                getLunarDate(getTotalDaySolar(_current->year, _current->mon, _current->day), _lunardate);
                _notify(this);
            }
        }
        else if (_mode == SETTING_MODE) {
            if (!_setting_time_modified) {
                Rtc::DateTime time = _rtc.dateTime();
                bool changed = false;
                if (time.sec != _current->sec) {
                    _current->sec = time.sec;
                    changed = true;
                }

                if (changed)
                    getLunarDate(getTotalDaySolar(_current->year, _current->mon, _current->day), _lunardate);
            }
            _notify(this);
        }

        return 0;
    }
};

#endif /* KAPPADATETIME_H */
