#include <Arduino.h>
#include "SerialProcessor.h"

SerialProcessor::SerialProcessor() : functionProcessor() {
  // set default function
  lastLen = 3;
  lastPkt[0] = 0x01;
  lastPkt[1] = 0x01;
  lastPkt[2] = lastPkt[0] ^ lastPkt[1];
}

bool SerialProcessor::checkNewPacketAvailable() { return Serial.peek() == SOF; }

void SerialProcessor::listen() {
  // Search for SOF
  while (Serial.available()) {
    if (Serial.peek() == SOF)
      break;
    Serial.read(); // drop noise byte
  }

  if (!Serial.available()) {
    // nothing left in buffer: replay last good
    functionProcessor.processPacket(lastPkt, lastLen, false);
    return;
  }

  Serial.read(); // consume SOF

  // Read LEN
  while (!Serial.available()) { /* wait */
  }

  uint8_t len = Serial.read();
  if (len < 3 || len > MAX_PKT_LEN) { // sanity check
    Serial.write(0x05);
    functionProcessor.processPacket(lastPkt, lastLen, false);
    return;
  }

  uint8_t packet[MAX_PKT_LEN];
  // Read the rest of the frame
  while (Serial.available() < len) {
    /* wait */
  }
  Serial.readBytes(reinterpret_cast<char*>(packet), len);

  uint8_t crc = SOF ^ len;
  for (uint8_t i = 0; i < len - 1; i++) {
    crc ^= packet[i];
  }

  if (crc != packet[len - 1]) {
    Serial.write(0x01); // Error: bad checksum
    functionProcessor.processPacket(lastPkt, lastLen, false);
    return;
  }

  memcpy(lastPkt, packet, len);
  lastLen = len;
  functionProcessor.processPacket(packet, len, true);
  return;
}