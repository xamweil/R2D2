"""
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
 *
 *   0x10-0x1F => Lid Specific errors
 *   0x20-0x2F => Nob Specific errors
 """

import Serial
import time
import struct


class SerialProcessor:
    def __init__(self, port, baud=57600):
        self.baud = baud
        self.port = port
        ser = serial.Serial(port, baud, timeout=1)
        
        # Protocoll
        self.SOF = 0xAA 
        self.CID = 0x00
        self.FID = 0x00

    def send_packet(self, payload, payload_len, rec_time, timeout=1):
        len = payload_len+3 # CID + FID + PAYLOAD + CRC
        frame = bytearray([self.SOF, len, self.CID, self.FID, payload])
        crc  = 0
        for b in frame:
            crc ^= b
        frame.append(crc)
        self.ser.write(frame)
        self.ser.timeout=timeout
        self.ser.read()


    


