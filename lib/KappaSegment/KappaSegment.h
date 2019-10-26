/* -*- mode: arduino; fill-column: 75; comment-column: 50 -*- */

#ifndef KAPPASEGMENT_H
#define KAPPASEGMENT_H

#include <Arduino.h>
#include "HT16K33.h"

class KappaSegment: public HT16K33 {

    const uint8_t I2C_ADDR{ 0x70 };
    const uint16_t NUM_TO_SEG[11] = { 		 //76543210
        0b01110111,	// 0
        0b00010010,	// 1
        0b01101011,	// 2
        0b01011011,	// 3
        0b00011110,	// 4
        0b01011101,	// 5
        0b01111101,	// 6
        0b00010111,	// 7
        0b01111111,	// 8
        0b01011111,	// 9
        0b00000000,   // NULL
    };

    const uint8_t brightness_tbl[3] = { 0, 8, 15  };

public:
    KappaSegment() : brightness(1) {}

    void setup() {
        init(I2C_ADDR);
    }

    void setBrightness(int level) {
        level = level & 0x3;
        if (brightness != level) {
            brightness = level;
            HT16K33::setBrightness(brightness_tbl[brightness]);
        }
    }

    void setNumber(uint8_t com, int number) {
        if (number < 0 || number > 99)
            return;

        com = com & 0x07;

        uint8_t tens = number / 10;
        uint8_t unit_of_digit = number % 10;

        /* exceptions */
        if (com == 3 || com == 6) {	/* month */
            tens = tens & 0x01;
            if (tens == 0) {
                setRow(com, NUM_TO_SEG[unit_of_digit] << 8 | tens);
                return;
            }
        } else if (com == 4 || com == 7) {	/* day */
            tens = tens & 0x03;
            if (tens == 0) {
                setRow(com, NUM_TO_SEG[unit_of_digit] << 8 | tens);
                return;
            }
        }

        setRow(com, NUM_TO_SEG[unit_of_digit] << 8 | NUM_TO_SEG[tens]);
    }

    void setNumber(uint8_t com, int digit1, int digit2) {
        if (digit1 < 0 || digit1 > 9)
            return;

        if (digit2 < 0 || digit2 > 9)
            return;

        com = com & 0x07;

        setRow(com, NUM_TO_SEG[digit1] << 8 | NUM_TO_SEG[digit2]);
    }

    void setTurnOff(uint8_t com) {
        setRow(com, 0);
    }

    void turnOffAll() {
        for (int i = 0; i < 8; i++)
            setTurnOff(i);
        write();
    }

private:
    uint8_t brightness;
};

#endif /* KAPPASEGMENT_H */
