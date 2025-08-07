#include <Arduino.h>

#include "SerialProcessor.h"
#include "FunctionProcessor.h"

SerialProcessor::SerialProcessor(FunctionProcessor& function_processor) : function_processor_(function_processor) { }

bool SerialProcessor::checkNewPacketAvailable() { return Serial.peek() == SOF_; }

void SerialProcessor::listen() {
    // Search for SOF
    while (Serial.available()) {
        if (Serial.peek() == SOF_)
            break;
        Serial.read(); // drop noise byte
    }

    if (!Serial.available()) {
        // nothing left in buffer: replay last good
        function_processor_.processPacket(last_pkt_, last_len_, false);
        return;
    }

    Serial.read(); // consume SOF

    // Read LEN
    while (!Serial.available()) { /* wait */
    }

    uint8_t len = Serial.read();
    if (len < 3 || len > MAX_PKT_LEN_) { // sanity check
        Serial.write(0x05);
        function_processor_.processPacket(last_pkt_, last_len_, false);
        return;
    }

    uint8_t packet[MAX_PKT_LEN_];
    // Read the rest of the frame
    while (Serial.available() < len) {
        /* wait */
    }
    Serial.readBytes(reinterpret_cast<char *>(packet), len);

    uint8_t crc = SOF_ ^ len;
    for (uint8_t i = 0; i < len - 1; i++) {
        crc ^= packet[i];
    }

    if (crc != packet[len - 1]) {
        Serial.write(0x01); // Error: bad checksum
        function_processor_.processPacket(last_pkt_, last_len_, false);
        return;
    }

    memcpy(last_pkt_, packet, len);
    last_len_ = len;
    function_processor_.processPacket(packet, len, true);
    return;
}
