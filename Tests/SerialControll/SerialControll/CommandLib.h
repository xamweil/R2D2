#ifndef COMMAND_LIB_H
#define COMMAND_LIB_H

#include <Arduino.h>

class CommandLib {
	
public:
	// Constructor: takes the Arduino pin number to control
	CommandLib(uint8_t pin);

	// Public method to process a command string
	void processCommand(String cmd);

private:
	uint8_t _pin;

	void commandOn();
	void commandOff();

};
#endif