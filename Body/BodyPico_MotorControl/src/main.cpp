#include "MotorControl.h"
#include "SerialProcessor.h"
#include "debug.h"

#include <Arduino.h>

#include <new>

static constexpr uint8_t PIN_SENSOR_HEAD = 28;

namespace {
std::array<std::byte, sizeof(MotorControl)>
    motor_control_buf alignas(MotorControl);
std::array<std::byte, sizeof(SerialProcessor)>
    serial_processor_buf alignas(SerialProcessor);
MotorControl *motor_control;
SerialProcessor *serial_processor;

void reference() {
    constexpr int32_t HOMING_SPEED = 750;

    motor_control->motors_.head.set_direction(false);
    motor_control->motors_.head.set_frequency(HOMING_SPEED);
    motor_control->motors_.head.set_enabled(true);

    while (digitalRead(PIN_SENSOR_HEAD) != LOW) {
        // keep moving until sensor triggers
    }

    while (digitalRead(PIN_SENSOR_HEAD) == LOW) {
        // keep stepping until sensor releases
    }

    motor_control->motors_.head.set_enabled(false);
    motor_control->motors_.head.set_frequency(0);
    // motor_control->motors_.head.reset_step_count();
}
} // namespace

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(PIN_SENSOR_HEAD, INPUT);

    // NOLINTBEGIN(cppcoreguidelines-owning-memory)
    // clang-format off
    motor_control = new (motor_control_buf.data()) MotorControl({
        .mid_foot = {
            .enable_pin = 2,     //  4
            .pulse_pin = 6,      //  9
            .direction_pin = 21, // 27
            .step_size = 1
        },
        .head = {
            .enable_pin = 3,     //  5
            .pulse_pin = 7,      // 10
            .direction_pin = 20, // 26
            .step_size = 1
        },
        .left_shoulder = {
            .enable_pin = 4,     //  6
            .pulse_pin = 8,      // 11
            .direction_pin = 19, // 25
            .step_size = 1
        },
        .right_shoulder = {
            .enable_pin = 4,     //  6
            .pulse_pin = 9,      // 12
            .direction_pin = 18, // 24
            .step_size = 1
        },
        .left_foot = {
            .enable_pin = 5,     //  7
            .pulse_pin = 10,     // 14
            .direction_pin = 17, // 22
            .step_size = 1
        },
        .right_foot = {
            .enable_pin = 5,     //  7
            .pulse_pin = 11,     // 15
            .direction_pin = 16, // 21
            .step_size = 1
        }
    });
    // clang-format on
    // NOLINTEND(cppcoreguidelines-owning-memory)

    Serial.begin(115200);
    while (!Serial) {
    }
    delay(100);

    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    serial_processor =
        new (serial_processor_buf.data()) SerialProcessor(motor_control);

    reference();

    DBG_PRINTLN("SETUP COMLETE");
}

void loop() {
    serial_processor->listen();
}
