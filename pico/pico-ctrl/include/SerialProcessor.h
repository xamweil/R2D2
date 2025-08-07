#ifndef SERIAL_PROCESSOR_H
#define SERIAL_PROCESSOR_H

#include <cstdint>

#include "FunctionProcessor.h"

struct SerialProcessor {
    static constexpr uint8_t SOF_ = 0xAA;
    static constexpr uint8_t MAX_PKT_LEN_ = 16;

    FunctionProcessor function_processor_;
    uint8_t last_pkt_[MAX_PKT_LEN_];
    uint8_t last_len_ = 0;

    SerialProcessor(FunctionProcessor& function_processor);

    void listen();
    static bool checkNewPacketAvailable();
};

#endif
