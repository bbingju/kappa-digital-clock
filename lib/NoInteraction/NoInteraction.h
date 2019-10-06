#ifndef NOINTERACTION_H
#define NOINTERACTION_H

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <VariableTimedAction.h>

class NoInteraction: public VariableTimedAction {

public:
    NoInteraction(): _interval(30 * 1000) {
        _callback = 0;
    }
    ~NoInteraction() {}

    void registerCallback(void (*callback)(void *arg)) {
        _callback = callback;
    }

    void restart() {
        if (isRunning()) {
            stop();
        }

        start(_interval, false);
    }

private:
    unsigned long _interval;

    void (*_callback)(void *);

    unsigned long run() {

        if (_callback) {
            _callback(this);
        }
        return 0;
    }
};

#endif /* NOINTERACTION_H */
