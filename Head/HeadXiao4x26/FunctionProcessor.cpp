#include <stdint.h>

#include "Matrix.h"
#include "FunctionProcessor.h"

FunctionProcessor::FunctionProcessor(): matrix(){}

uint8_t FunctionProcessor::processPacket(uint8_t *packet, uint8_t len) {
  uint8_t classID = packet[0];
  uint8_t funcID = packet[1];
  uint8_t payloadLen = len - 3;

  const uint8_t *payload = &packet[2];

  if (classID != 0x01) {
    return 0x02;
  }

  switch(funcID) {
    case 0x01:
      // TODO: random lights
      return 0x00;
    case 0x02: // 
      // TODO: display off
      return 0x00;
    case 0x03:
      // TODO: display on
      return 0x00;
    case 0x04:
      if (payloadLen < 4) return 0x04;

      bool frame[4][26];

      for (uint8_t row = 0; row < 4; ++row) {
        uint8_t byte = payload[row];
        for (uint8_t col = 0; col < 24; ++col) {
          frame[row][col] = (byte & (1 << col)) != 0;
        }
      }

      matrix.setLeds(frame);
      return 0x00;
    case 0x05:
      if (payloadLen != 3) return 0x04;
      // TODO: set single led

      return 0x00;
    default:
      return 0x03;
  }

}
