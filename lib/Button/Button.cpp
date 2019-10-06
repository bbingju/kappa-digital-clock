#include "Button.h"

void Button::registerCallback(void (*callback)(void *, Button::EVENT))
{
    _event_callback = callback;
}

boolean Button::isPushing()
{
    return !digitalRead(_pin);
}
