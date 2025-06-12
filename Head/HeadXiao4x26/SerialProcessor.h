#ifndef SERIAL_PROCESSOR_H
#define SERIAL_PROCESSOR_H

#include "FunctionProcessor.h"

class SerialProcessor {
public:
    SerialProcessor();
    void listen();
    static bool checkNewPacketAvailable();
    FunctionProcessor functionProcessor;
private:
    static constexpr uint8_t SOF = 0xAA;
    static constexpr uint8_t  MAX_PKT_LEN = 16;
    uint8_t lastPkt[MAX_PKT_LEN];
    uint8_t lastLen = 0;
};

#endif
