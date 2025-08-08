#include <Arduino.h>
#include <cstdint>

#include "FunctionProcessor.h"
#include "MotorControl.h"
#include "SerialProcessor.h"
#include "debug.h"

// builtin led is 25
static constexpr uint8_t LED_PIN = 2;

// clang-format off
MotorControl motors[] = {
    // motor 0
    MotorControl({
        .enable_pin = 2,    // physical pin 4
        .direction_pin = 3, // physical pin 5
        .pulse_pin = 4      // physical pin 6
    }),
};
// clang-format on
static constexpr uint8_t NUM_MOTORS = 1;

FunctionProcessor function_processor(motors, NUM_MOTORS);
SerialProcessor serial_processor(function_processor);

void ledBlink() {
    // digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    // digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}

void setup() {
    // pinMode(LED_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial) {
    }

    delay(100);

    DBG_PRINTLN("SETUP COMLETE");
}

void loop() {
    // serial_processor.listen();
    ledBlink();
    motors[0].move();
}
