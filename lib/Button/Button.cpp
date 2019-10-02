#include <Wire.h>
#include "Button.h"

boolean Button::isPushed()
{
	reading = digitalRead(pin);

	if (reading != last_state) {
		last_debounce_time = millis();
	}

	if (millis() - last_debounce_time > debounce_delay) {

		if (reading != state) {
			state = reading;

			if (state == LOW) {
				return true;
			}
		}
	}
	last_state = reading;
	return false;
}

boolean Button::isPushing()
{
  return !digitalRead(pin);
}
