#include <Arduino.h>
#include "SerialProcessor.h"

SerialProcessor serialProcessor;

void setup()
{
    Serial.begin(115200);

    serialProcessor.setup();

}

void loop()
{
    // Handle incoming commands
    serialProcessor.process();

    // Run motor controllers
    serialProcessor.updateMotors();
}