//
//
//

#include "lunar_cal.h"

static int lunarDayOfMonth[] = { 29,30,58,59,59,60 };

static int lunarDayOfMonthFrac[] = { 0,0,29,30,30,30 };

static uint8_t lunarDayOfMonthIndex[][12] = {
        // 2001 ~ 2010
        {1,1,1,2,1,0,0,1,0,1,0,1},{1,1,0,1,0,1,0,0,1,0,1,0},
        {1,1,0,1,1,0,1,0,0,1,0,1},{0,4,1,1,0,1,0,1,0,1,0,1},
        {0,1,0,1,0,1,1,0,1,1,0,0},{1,0,1,0,1,0,4,1,1,0,1,1},
        {0,0,1,0,0,1,0,1,1,1,0,1},{1,0,0,1,0,0,1,0,1,1,0,1},
        {1,1,0,0,4,0,1,0,1,0,1,1},{1,0,1,0,1,0,0,1,0,1,0,1},
        // 2011 ~ 2020
        {1,0,1,1,0,1,0,0,1,0,1,0},{1,0,5,1,0,1,0,0,1,0,1,0},
        {1,0,1,1,0,1,0,1,0,1,0,1},{0,1,0,1,0,1,0,1,4,1,0,1},
        {0,1,0,0,1,0,1,1,1,0,1,1},{0,0,1,0,0,1,0,1,1,0,1,1},
        {1,0,0,1,2,1,0,1,0,1,1,1},{0,1,0,1,0,0,1,0,1,0,1,1},
        {1,0,1,0,1,0,0,1,0,1,0,1},{1,0,1,4,1,0,0,1,0,1,0,1},
        // 2021 ~ 2030
        {0,1,1,0,1,0,1,0,1,0,1,0},{1,0,1,0,1,1,0,1,0,1,0,1},
        {0,4,1,0,1,0,1,1,0,1,0,1},{0,1,0,0,1,0,1,1,0,1,1,0},
        {1,0,1,0,0,4,1,0,1,1,1,0},{1,0,1,0,0,1,0,1,0,1,1,1},
        {0,1,0,1,0,0,1,0,0,1,1,1},{0,1,1,0,4,0,1,0,0,1,1,0},
        {1,1,0,1,1,0,0,1,0,0,1,1},{0,1,0,1,1,0,1,0,1,0,1,0},
        // 2031 ~ 2040
        {1,0,4,1,0,1,1,0,1,0,1,0},{1,0,0,1,0,1,1,0,1,1,0,1},
        {0,1,0,0,1,0,4,1,1,1,0,1},{0,1,0,0,1,0,1,0,1,1,1,0},
        {1,0,1,0,0,1,0,0,1,1,0,1},{1,1,0,1,0,3,0,0,1,0,1,1},
        {1,1,0,1,0,0,1,0,0,1,0,1},{1,1,0,1,0,1,0,1,0,0,1,0},
        {1,1,0,1,4,1,0,1,0,1,0,0},{1,0,1,1,0,1,1,0,1,0,1,0},
        // 2041 ~ 2050
        {1,0,0,1,0,1,1,0,1,1,0,1},{0,4,0,1,0,1,0,1,1,0,1,1},
        {0,1,0,0,1,0,0,1,1,0,1,1},{1,0,1,0,0,1,2,1,0,1,1,1},
        {1,0,1,0,0,1,0,0,1,0,1,1},{1,0,1,1,0,0,1,0,0,1,0,1},
        {1,0,1,1,3,0,1,0,0,1,0,1},{0,1,1,0,1,1,0,1,0,0,0,0},
        {1,0,1,0,1,1,0,1,1,0,1,0},{1,0,3,0,1,0,1,1,0,1,1,0}
};

static int lunarDayOfYear[] = {
        384,354,355,384,354,385,354,354,384,354,    // 2001 ~ 2010
        354,384,355,384,355,354,384,354,354,384,    // 2011 ~ 2020
        354,355,384,354,384,355,354,383,355,354,    // 2021 ~ 2030
        384,355,384,354,354,384,354,354,384,355,    // 2031 ~ 2040
        355,384,354,384,354,354,384,353,355,384     // 2041 ~ 2050
};

static uint8_t solarDayNum[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

int getSolarDayOfMonth(int y, int m) {    // 월별 일수 계산
        if (m != 2) return solarDayNum[m - 1];

        if ((y % 400 == 0) || ((y % 100 != 0) && (y % 4 == 0))) return 29;
        else return 28;
}

int getSolarDayOfYear(int y) {
        if ((y % 400 == 0) || ((y % 100 != 0) && (y % 4 == 0))) return 366;
        else return 365;
}

// 양력 2001/01/24 = 음력 2001/01/01
uint8_t solarBasis[] = { 1, 1,24 };

int getTotalDaySolar(uint8_t y, uint8_t m, uint8_t d) {
        int i, ret = 0;

        for (i = solarBasis[0]; i < y; i++) ret += getSolarDayOfYear(i);
        for (i = 1; i < m; i++) ret += getSolarDayOfMonth(y, i);
        ret += d;
        for (i = 1; i < solarBasis[1]; i++) ret -= getSolarDayOfMonth(solarBasis[0], i);
        ret -= solarBasis[2];

        return ret;
}

void getLunarDate(int totalDay, uint8_t* result) {    // result[0]~[2] 에 넣음
        int y = -1, m = -1, d = 0, f;

        while (totalDay >= lunarDayOfYear[++y]) totalDay -= lunarDayOfYear[y];
        while (totalDay >= (d = lunarDayOfMonth[lunarDayOfMonthIndex[y][++m]]))
                totalDay -= d;
        d = totalDay;

        f = lunarDayOfMonthFrac[lunarDayOfMonthIndex[y][m]];
        if (d >= f) d -= f;

        result[0] = y + 1; result[1] = m + 1; result[2] = d + 1;
}

int isLeapYear(int year)
{
  if (year % 400 == 0)
    return 1;
  else if (year % 100 == 0)
    return 0;
  else if (year % 4 == 0)
    return 1;
  else
    return 0;
}

uint8_t getLastDayOfMonth(uint8_t y, uint8_t m)
{
  if (m < 1 || m > 12)
    return 30;

  int year = 2000 + y;
  uint8_t days = solarDayNum[m - 1];
  if (isLeapYear(year) && m == 2)
    days++;
  return days;
}

uint8_t getDayOfWeek(uint8_t y, uint8_t m, uint8_t d)
{
  int year = y + 2000;
  return (d += m < 3 ? year-- : year - 2, 23*m/9 + d + 4 + year/4- year/100 + year/400)%7;
}
