#include <Arduino.h>
#include "FunctionProcessor.h"

FunctionProcessor::FunctionProcessor(): ledMatrix(){}

void FunctionProcessor::processPacket(uint8_t *packet, uint8_t len, bool newPacket) {
  uint8_t classID = packet[0];
  uint8_t funcID = packet[1];
  uint8_t payloadLen = len - 3;

  const uint8_t *payload = &packet[2];

  if (classID != 0x01) {
    Serial.write(0x02);
    return;
  }

  uint8_t ret = 0x00;

  switch(funcID) {
    case 0x01: {
      uint8_t rateOn = 35;
      uint8_t rateChange = 10;
      uint8_t dt = 150;

      if (payloadLen >= 1) rateOn     = payload[0];
      if (payloadLen >= 2) rateChange = payload[1];
      if (payloadLen >= 3) dt         = payload[2];

      if (newPacket)
        Serial.write(ret);
      ledMatrix.randomLights(rateOn, rateChange, dt);
      break;
    }
    case 0x02:
      if (newPacket)
        Serial.write(ret);
      ledMatrix.displayOff();
      break;
    case 0x03:
      if (newPacket)
        Serial.write(ret);
      ledMatrix.displayOn();
      break;
    case 0x04:
      if (payloadLen < 16) {
        ret = 0x04;
        Serial.write(ret);
        break;
      }

      if (newPacket)
        Serial.write(ret);
      bool frame[4][26];

      for (uint8_t i = 0; i < 4; i++) {
        uint32_t row = 0;
        row |= static_cast<uint32_t>(payload[i * 4]);
        row |= static_cast<uint32_t>(payload[i * 4 + 1]) << 8;
        row |= static_cast<uint32_t>(payload[i * 4 + 2]) << 16;
        row |= static_cast<uint32_t>(payload[i * 4 + 3]) << 24;
      
        for (uint8_t j = 0; j < 26; j++) {
          frame[i][j] = (row & (1 << j)) != 0;
        }
      }

      ledMatrix.setLeds(frame);
      break;
    default:
      ret = 0x03;
      Serial.write(ret);
      break;
  }

}
