#ifndef FUNCTION_PROCESSOR_H
#define FUNCTION_PROCESSOR_H

#include <Arduino.h>
#include <cstdint>

#include "MotorControl.h"

struct FunctionProcessor {
    MotorControl* motors_;
    uint8_t num_motors_;

    FunctionProcessor(MotorControl* motors, const uint8_t num_motors);

    void processPacket(uint8_t *packet, uint8_t len, bool new_packet);
};

#endif
