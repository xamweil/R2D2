#include "SerialUSB.h"
#include <Arduino.h>

const int ENA_PIN = 6;
const int DIR_PIN = 4;
const int PUL_PIN = 5;

const int LED_PIN = 25;

/**
 * 6 = ENA
 * 4 = DIR
 * 5 = PUL
 */

void setup() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  digitalWrite(ENA_PIN, LOW);

  Serial.begin(115200);
}

int Index;

void loop() {
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("FIRST LOOP\n");
  for (Index = 0; Index < 5000; Index++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(1000);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.print("SECOND LOOP\n");
  for (Index = 0; Index < 5000; Index++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
