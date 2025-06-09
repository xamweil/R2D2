#include <Arduino.h>
#include <stdint.h>

#include "SerialProcessor.h"
#include "FunctionProcessor.h"

SerialProcessor::SerialProcessor():
  functionProcessor() {
    //set default function
    lastLen = 3;
    lastPkt[0] = 0x01;
    lastPkt[1] = 0x01;
    lastPkt[2]      = lastPkt[0] ^ lastPkt[1];
  }

uint8_t SerialProcessor::listen(){
  // Search for SOF
  while (Serial.available()) {
    if (Serial.peek() == SOF) break;
    Serial.read();           // drop noise byte
  }
  if (!Serial.available()) {
    // nothing left in buffer: replay last good
    return functionProcessor.processPacket(lastPkt, lastLen);
  }

  // Consume SOF
  Serial.read();

  // Read LEN
  while (Serial.available() < 1) { /* wait */ }
  uint8_t len = Serial.read();
  if (len < 3 || len > MAX_PKT){   // sanity check
    Serial.write(0x05);       // sanity check failed
    return functionProcessor.processPacket(lastPkt, lastLen);
  }

  uint8_t packet[MAX_PKT];
  // Read the rest of the frame
  while (Serial.available() < len) { /* wait */ }
  Serial.readBytes(reinterpret_cast<char *>(packet), len);
  uint8_t crc = SOF ^ len; 
  for (uint8_t i = 0; i < len - 1; i++) {
    crc ^= packet[i];
  } 
  if (crc!=packet[len-1]) { 
    Serial.write(0x01); //Error: bad checksum
    return functionProcessor.processPacket(lastPkt, lastLen);
  }
  memcpy(lastPkt, packet, len);
  lastLen = len;
  
  
  return functionProcessor.processPacket(packet, len);
}