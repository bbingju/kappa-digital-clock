/* -*- mode: arduino; fill-column: 75; comment-column: 50 -*- */

#ifndef DHT_H
#define DHT_H

#include <VariableTimedAction.h>

class DHT: public VariableTimedAction {
public:
    DHT(): _pin(A1), _current(0.f) {}
    ~DHT() {}

    float getTemperature() {
        return _current;
    }

private:
    int _pin;
    float _current;

    unsigned long run() {
        float t = readTemperature();
        if (isnan(t)) {
            Serial.println("Error reading temperature!");
        }
        else {
            _current = t;
        }

        return 0;
    }

    float readTemperature() {
        static float Rt,Adc,T,LnRt;
        static float Ra = 0.00127225169280943;
        static float Rb = 0.000238650944540013;
        static float Rc = 0.0000000794319275047443;

        Adc = (float) analogRead(_pin);
        Rt = (10000.0 * Adc)/(1024.0 - Adc);
        LnRt = log(Rt);
        T = (1.0/(Ra + Rb*LnRt + Rc*pow(LnRt, 3.0)))-273.15;
        // Serial.print("Temperature: ");
        // Serial.print(T);
        // Serial.println(" *C");
        return T;
    }
};

#endif /* DHT_H */
