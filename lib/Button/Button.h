#ifndef BUTTON_H
#define BUTTON_H

// include appropriate version of Arduino code
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
  
class Button {
public:
    Button(int pin) {
        _pin = pin;
        pinMode(_pin, INPUT_PULLUP);
    };
    ~Button() {};

    boolean isPushed();
    boolean isPushing();

private:
    int _pin;
    const uint32_t debounce_delay = 50;
    int reading;
    int last_state = HIGH;
    int state = HIGH;
    uint32_t last_debounce_time;
};

#endif /* BUTTON_H */
