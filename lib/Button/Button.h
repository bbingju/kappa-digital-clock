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
	Button(int p) { pin = p; };
	~Button() {};

	int pin;
	boolean isPushed();
	boolean isPushing();

 private:
	const long debounce_delay = 50;
	int reading;
	int last_state = HIGH;
	int state = HIGH;
	uint32_t last_debounce_time;
};

#endif /* BUTTON_H */
