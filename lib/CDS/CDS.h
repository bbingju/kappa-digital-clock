#ifndef CDS_H
#define CDS_H

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <VariableTimedAction.h>

class CDS: public VariableTimedAction {
public:
    CDS(): _pin(A0), _brightness(0) { }
    ~CDS() {}

    int getBrightness() {
        return _brightness;
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
            else if (avg > 900)
                _brightness = 0;

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
