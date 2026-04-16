#include "button.h"

Button::Button(uint8_t pin, uint8_t button_id, unsigned long debounce_ms)
: pin_(pin),
  button_id_(button_id),
  debounce_ms_(debounce_ms),
  stable_state_(HIGH),
  last_raw_state_(HIGH),
  pressed_event_(false),
  last_change_ms_(0) {}

void Button::begin() {
  pinMode(pin_, INPUT_PULLUP);
  stable_state_ = digitalRead(pin_);
  last_raw_state_ = stable_state_;
  pressed_event_ = false;
  last_change_ms_ = millis();
}

void Button::update() {
  const bool raw_state = digitalRead(pin_);
  const unsigned long now = millis();

  if (raw_state != last_raw_state_) {
    last_raw_state_ = raw_state;
    last_change_ms_ = now;
  }

  if ((now - last_change_ms_) >= debounce_ms_ && stable_state_ != raw_state) {
    stable_state_ = raw_state;

    // LOW is pressed
    if (stable_state_ == LOW) {
      pressed_event_ = true;
    }
  }
}

bool Button::wasPressed() {
    if (pressed_event_) {
        pressed_event_ = false;
        return true;
    }
    return false;
}

uint8_t Button::buttonId() const {
  return button_id_;
}
