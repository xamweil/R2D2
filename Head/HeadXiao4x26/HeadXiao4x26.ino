#include <Arduino.h>
#include <stdint.h>
#include "SerialProcessor.h"

SerialProcessor serialProcessor;

void setup() {
  Serial.begin(57600);
  delay(500);
}

void loop() {
  uint8_t ret = 0xFF;

  while (ret == 0xFF) {
    ret = serialProcessor.listen();

    // TODO: is this needed?
    if (ret == 0xFF) {
      delay(1);
    }
  }

  Serial.write(ret);
}
