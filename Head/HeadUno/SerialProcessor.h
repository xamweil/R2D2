/*
 * ┌──────┬──────┬────────┬────────────────────────────────────────────┐
 * │ Byte │ Name │ Size   │ Description                                │
 * ├──────┼──────┼────────┼────────────────────────────────────────────┤
 * │ 0    │ SOF  │ 1 byte │ 0xAA start-of-frame marker                 │
 * │ 1    │ LEN  │ 1 byte │ length of (ClassID + FuncID + Payload + CRC) │
 * │ 2    │ CID  │ 1 byte │ Class ID (object type)                     │
 * │ 3    │ FID  │ 1 byte │ Func ID (method within that class)         │
 * │ 4…N  │ DATA │ LEN-3  │ Payload bytes (parameters, 0…LEN-3 bytes)  │
 * │ N+1  │ CRC  │ 1 byte │ XOR checksum of bytes 0…N                  │
 * └──────┴──────┴────────┴────────────────────────────────────────────┘
 *
 * Total frame size = 2 (SOF+LEN) + LEN
 *
 * Example: Set lid #3 open to 90° (uint16_t payload)
 *   SOF = 0xAA
 *   LEN = 1+1+2+1 = 5
 *   CID = 0x01 (LID)
 *   FID = 0x03 (setOpenPos)
 *   DATA = 0x00 0x5A
 *   CRC = XOR(0xAA,0x05,0x01,0x03,0x00,0x5A)
 *
 * Return sig:
 *   0x00 = OK
 *   0x01 = bad checksum
 *   0x02 = unknown class
 *   0x03 = unknown function
 *   0x04 = bad payload
 *   0x05 = sanity check failed. Either packet is too long or len<3 (min packet size)
 *   0x06 = Unknown error
 *
 *   0x10-0x1F => Lid Specific errors
 *   0x20-0x2F => Nob Specific errors
 *   0x30-0x3F => CameraTilt Specific errors
 *   -  0x31 : Value out of bound
 */
#ifndef SERIAL_PROCESSOR_H
#define SERIAL_PROCESSOR_H

#include <Arduino.h>
#include "Lid.h"
#include "Nob.h"
#include "CameraTiltActuator.h"
#include <Adafruit_PWMServoDriver.h>

class SerialProcessor{
  public:
    SerialProcessor(Adafruit_PWMServoDriver &driver1, Adafruit_PWMServoDriver &driver2);

    uint8_t listen();
    uint8_t processPacket(uint8_t *packet, uint8_t len);
    void updateAll();



  private:
    Adafruit_PWMServoDriver _driver1;
    Adafruit_PWMServoDriver _driver2;

    uint16_t _servoMin;
    uint16_t _servoMax;

    //Lids
    Lid lid1_1;
    Lid lid1_2;
    Lid lid1_3;
    Lid lid1_4;
    Lid lid1_5;
    Lid lid1_6;

    Lid lid2_1;
    Lid lid2_2;
    Lid lid2_3;
    Lid lid2_4;
    Lid lid2_5;

    //Nobs
    Nob nob1;
    Nob nob2;
    Nob nob3;

    //Camera Tilt Servo
    CameraTiltActuator cta;
    
    //Protocoll
    uint8_t SOF; // Start of frame 0xAA
    uint8_t MAX_PKT = 32;
    




  uint8_t processpacket(const uint8_t *packet, uint8_t len);


};

#endif
