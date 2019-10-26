/* -*- mode: arduino; fill-column: 75; comment-column: 50 -*- */

#ifndef CDS_H
#define CDS_H

#include <VariableTimedAction.h>

class CDS: public VariableTimedAction {
public:
    CDS(): _pin(A0), _brightness(2) { }
    ~CDS() {}

    int getBrightness() {
        return _brightness;
    }
    void setBrightness(int brightness) {
        _brightness = brightness;
    }

private:
    int _pin;
    int _brightness;

    unsigned long run() {
        static int total = 0, avg = 0;
        static int count = 0;
        int cds = analogRead(_pin);

        if (count >= 4) {
            avg = total / count;

            // Serial.print("CDS Average: ");
            // Serial.println(avg);

            if (avg < 400)
                _brightness = 2;
            else if (avg > 400 && avg < 900)
                _brightness = 1;
            else if (avg > 900 && avg < 1020)
                _brightness = 0;
            else if (avg >= 1020 && avg < 1024)
                _brightness = 2;

            count = 0;
            total = 0;
        }
        else {
            total += cds;
            count++;
        }

        return 0;
    }
};

#endif /* CDS_H */
