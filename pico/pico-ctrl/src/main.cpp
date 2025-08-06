#include <Arduino.h>

const int LED_PIN = LED_BUILTIN;

void setup() {
  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH); // Turn LED on
  delay(1000);                 // Wait 1 second (1000 milliseconds)
  digitalWrite(LED_PIN, LOW);  // Turn LED off
  delay(1000);                 // Wait 1 second
}
