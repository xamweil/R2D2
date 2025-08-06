#include <Arduino.h>
#include "SerialProcessor.h"

SerialProcessor serialProcessor;

void setup() {
  Serial.begin(57600);
  delay(500);
}

void loop() {
  serialProcessor.listen();
}
