#include "FunctionProcessor.h"
#include "MotorControl.h"
#include <Arduino.h>

FunctionProcessor::FunctionProcessor(MotorControl *motors, const uint8_t num_motors)
    : motors_(motors), num_motors_(num_motors) {}

void FunctionProcessor::processPacket(uint8_t *packet, uint8_t len,
                                      bool new_packet) {}
