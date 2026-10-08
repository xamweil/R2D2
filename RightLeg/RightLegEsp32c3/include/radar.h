#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ld2410.h>


struct RadarReading {
  bool connected = false;
  bool presence = false;

  uint16_t moving_distance_cm = 0;
  uint8_t moving_energy = 0;

  uint16_t stationary_distance_cm = 0;
  uint8_t stationary_energy = 0;
};

class Radar {
public:
  Radar(HardwareSerial& serial,
        uint8_t rx_pin,
        uint8_t tx_pin,
        uint32_t radar_baud = 256000);

  void begin();
  void update();

  bool isConnected() const;
  RadarReading getReading() const;

private:
  HardwareSerial& serial_;
  uint8_t rx_pin_;
  uint8_t tx_pin_;
  uint32_t radar_baud_;

  ld2410 radar_;
  RadarReading reading_;
};