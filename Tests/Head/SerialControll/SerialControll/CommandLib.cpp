#include "CommandLib.h"

CommandLib::CommandLib(uint8_t pin) : _pin(pin) {
	pinMode(_pin, OUTPUT);
}

void CommandLib::processCommand(String cmd) {
	if (cmd == "ON") {
		commandOn();
	}
	else if (cmd == "OFF") {
		commandOff();
	}
	
}

void CommandLib::commandOn() {
	digitalWrite(_pin, HIGH);
}

void CommandLib::commandOff() {
	digitalWrite(_pin, LOW);
}