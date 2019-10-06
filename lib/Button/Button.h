#ifndef BUTTON_H
#define BUTTON_H

// include appropriate version of Arduino code
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <VariableTimedAction.h>

class Button: public VariableTimedAction {
public:
    Button(int pin) {
        _pin = pin;
        _event_callback = 0;
        pinMode(_pin, INPUT_PULLUP);
    }
    ~Button() {}

    enum EVENT {
        DOWN,
        UP,
        PUSHING,
    };

    void registerCallback(void (*)(void *, EVENT));
    boolean isPushing();

private:
    int _pin;
    const uint32_t debounce_delay = 50;
    const uint32_t pushing_delay = 200;
    const uint32_t first_pushing_delay = 1600;
    int _pushing_count = 0;
    int _reading;
    int _last_state = HIGH;
    int _state = HIGH;
    uint32_t last_debounce_time;
    void (*_event_callback)(void *, EVENT) = 0;
    

    unsigned long run() {
        _reading = digitalRead(_pin);

        if (_reading != _last_state) {
            last_debounce_time = millis();
        }

        if (millis() - last_debounce_time > debounce_delay) {

            if (_reading != _state) {
                _state = _reading;

                if (_state == LOW) {
                    // Serial.println("_state == LOW");
                    _pushing_count = 0;
                }
                else if (_state == HIGH) {
                    // Serial.print("_state == HIGH ");
                    // Serial.println(_pushing_count, DEC);
                    if (_event_callback && _pushing_count < 1) {
                        _event_callback(this, UP);
                    }
                    _pushing_count = 0;
                }
            }
            else if (_reading == LOW && _state == LOW) {

                uint32_t delay = (_pushing_count == 0) ? first_pushing_delay : pushing_delay;

                if (millis() - last_debounce_time > delay) {

                        // Serial.println("PUSHING");

                        if (_event_callback) {
                            _event_callback(this, PUSHING);
                        }
                        _pushing_count++;
                        last_debounce_time = millis();
                }
            }
        }
        _last_state = _reading;
        return 0;
    }
};

#endif /* BUTTON_H */
