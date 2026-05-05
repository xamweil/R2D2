#pragma once

#include <Arduino.h>

class Button{
    public:
        Button(uint8_t pin, uint8_t button_id, unsigned long debounce_ms = 30);

        void begin();
        void update();

        bool wasPressed();
        uint8_t buttonId() const;

    private:
        uint8_t pin_;
        uint8_t button_id_;
        unsigned long debounce_ms_;

        bool stable_state_;
        bool last_raw_state_;
        bool pressed_event_;
        unsigned long last_change_ms_;

};