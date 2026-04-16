#include "radar.h"

Radar::Radar(HardwareSerial& serial, uint8_t rx_pin, uint8_t tx_pin, uint32_t radar_baud): 
        serial_(serial), rx_pin_(rx_pin), tx_pin_(tx_pin),  radar_baud_(radar_baud) {}

void Radar::begin() {
    serial_.begin(radar_baud_, SERIAL_8N1, rx_pin_, tx_pin_);
    delay(300);

    reading_.connected = radar_.begin(serial_);
}

void Radar::update() {
  radar_.read();

  reading_.connected = radar_.isConnected();
  reading_.presence = radar_.presenceDetected();
  reading_.moving_distance_cm = radar_.movingTargetDistance();
  reading_.moving_energy = radar_.movingTargetEnergy();
  reading_.stationary_distance_cm = radar_.stationaryTargetDistance();
  reading_.stationary_energy = radar_.stationaryTargetEnergy();
}

bool Radar::isConnected() const {
  return reading_.connected;
}

RadarReading Radar::getReading() const {
  return reading_;
}