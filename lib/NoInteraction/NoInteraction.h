/* -*- mode: arduino; fill-column: 75; comment-column: 50 -*- */

#ifndef NOINTERACTION_H
#define NOINTERACTION_H

#include <VariableTimedAction.h>

class NoInteraction: public VariableTimedAction {

public:
    NoInteraction(): _interval(10 * 1000) {
        _callback = 0;
    }
    ~NoInteraction() {}

    void registerCallback(void (*callback)(void *arg)) {
        _callback = callback;
    }

    void reset() {
        if (isRunning()) {
            stop();
        }

        start(_interval, false);
    }

private:
    const unsigned long count_max = 6;
    unsigned long _interval;
    unsigned long _count = 0;

    void (*_callback)(void *);

    unsigned long run() {

        if (_callback && (++_count >= count_max)) {
            _count = 0;
            _callback(this);
        }
        return 0;
    }
};

#endif /* NOINTERACTION_H */
