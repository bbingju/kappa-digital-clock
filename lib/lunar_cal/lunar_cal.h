// lunar_cal.h

#ifndef _LUNAR_CAL_h
#define _LUNAR_CAL_h

/* #if defined(ARDUINO) && ARDUINO >= 100 */
/* 	#include "arduino.h" */
/* #else */
/* 	#include "WProgram.h" */
/* #endif */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    int getSolarDayOfMonth(int y, int m);
    int getSolarDayOfYear(int y);
    int getTotalDaySolar(uint8_t y, uint8_t m, uint8_t d);
    void getLunarDate(int totalDay, uint8_t* result);
    uint8_t getLastDayOfMonth(uint8_t y, uint8_t m);
    uint8_t getDayOfWeek(uint8_t y, uint8_t m, uint8_t d);

#ifdef __cplusplus
}
#endif

#endif
