#include <Arduino.h>

#include "LedMatrix.h"
#include "SerialProcessor.h"

LedMatrix::LedMatrix() {
  pinMode(SRCLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SER, OUTPUT);
  digitalWrite(SRCLK, LOW);
  digitalWrite(RCLK, LOW);
  digitalWrite(SER, LOW);
}

void LedMatrix::writeToRegister(const int bits[30]) {
  for (int i = 29; i >= 0; i--) {
    digitalWrite(SER, bits[i]);
    digitalWrite(SRCLK, HIGH);
    digitalWrite(SRCLK, LOW);
  }

  digitalWrite(RCLK, HIGH);
  digitalWrite(RCLK, LOW);
}

void LedMatrix::resetLedBuffer() {
  for (int i = 0; i < 30; i++) {
    ledBuffer[i] = 1;
  }
  writeToRegister(ledBuffer);
}

void LedMatrix::randomLights(uint8_t ratOn, uint8_t ratChange, uint8_t dt_random) {
  bool leds[4][26];
  unsigned long lastUpdate = millis();

  // set ratOn% of the LED's on
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 26; j++) {
      leds[i][j] = (random(100) < ratOn);
    }
  }

  while (true) {
    // ratChange% of LED's are changed every iteration
    for (int i = 0; i < 4; i++) {
      ledBuffer[i] = 0;

      for (int j = 4; j < 30; j++) {
        ledBuffer[j] = !leds[i][j];
      }

      writeToRegister(ledBuffer);
      delay(dt);
      ledBuffer[i] = 1;
    }

    if (millis() - lastUpdate >= dt_random) {
      lastUpdate = millis();
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 26; j++) {
          if (random(100) < ratChange)
            leds[i][j] = !leds[i][j];
        }
      }
    }

    if (SerialProcessor::checkNewPacketAvailable()) {
      resetLedBuffer();
      break;
    }
  }
}

void LedMatrix::displayOff() {
  for (int i = 4; i < 30; i++) {
    ledBuffer[i] = 1;
  }

  for (int i = 0; i < 4; i++) {
    ledBuffer[i] = 0;

    writeToRegister(ledBuffer);
    delay(dt);
    ledBuffer[i] = 1;
  }

  while(true) {
    // TODO: how long to wait here?
    delay(dt);
    if (SerialProcessor::checkNewPacketAvailable()) {
      resetLedBuffer();
      break;
    }
  }
}

void LedMatrix::displayOn() {
  for (int i = 4; i < 30; i++) {
    ledBuffer[i] = 0;
  }

  while (true) {
    for (int i = 0; i < 4; i++) {
      ledBuffer[i] = 0;

      writeToRegister(ledBuffer);
      delay(dt);
      ledBuffer[i] = 1;
    }

    if (SerialProcessor::checkNewPacketAvailable()) {
      resetLedBuffer();
      break;
    }
  }
}

void LedMatrix::setLeds(bool frame[4][26]) {
  while (true) {
    for (int i = 0; i < 4; i++) {
      ledBuffer[i] = 0;
      for (int j = 4; j < 30; j++) {
        ledBuffer[j] = !frame[i][j];
      }

      writeToRegister(ledBuffer);
      delay(dt);
      ledBuffer[i] = 1;
    }

    if (SerialProcessor::checkNewPacketAvailable()) {
      resetLedBuffer();
      break;
    }
  }
}
