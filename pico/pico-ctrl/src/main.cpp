#include <Arduino.h>
#include <cstdint>

#include "FunctionProcessor.h"
#include "MotorControl.h"
#include "SerialProcessor.h"
#include "debug.h"

static constexpr uint8_t LED_PIN = 25;

// clang-format off
MotorControl motors[] = {
    // motor 0
    MotorControl({
        .enable_pin = 6,    // physical pin 9
        .direction_pin = 4, // physical pin 6
        .pulse_pin = 5      // physical pin 7
    }),

    // motor 1
    // MotorControl({
    //     .ena_pin_ = 6,
    //     .dir_pin_ = 5,
    //     .pul_pin_ = 5
    // })
    //
};
// clang-format on
static constexpr uint8_t NUM_MOTORS = 1;

FunctionProcessor function_processor(motors, NUM_MOTORS);
SerialProcessor serial_processor(function_processor);

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.begin(115200);
    while (!Serial) {
    }

    DBG_PRINTLN("SETUP COMLETE");
}

void loop() {
    // serial_processor.listen();
    motors[0].move();
}
