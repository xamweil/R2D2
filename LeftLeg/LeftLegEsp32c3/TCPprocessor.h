/*"""
 * ┌──────┬──────┬────────┬────────────────────────────────────────────┐
 * │ Byte │ Name │ Size   │ Description                                │
 * ├──────┼──────┼────────┼────────────────────────────────────────────┤
 * │ 0    │ SOF  │ 1 byte │ 0xAA start-of-frame marker                 │
 * │ 1    │ LEN  │ 1 byte │ length of (ClassID + FuncID + Payload + CRC) │
 * │ 2    │ CID  │ 1 byte │ Class ID (object type)                     │
 * │ 3    │ FID  │ 1 byte │ Func ID (method within that class)         │
 * │ 4…N  │ DATA │ LEN-3  │ Payload bytes (parameters, 0…LEN-3 bytes)  │ 
 * └──────┴──────┴────────┴────────────────────────────────────────────┘
 *
 * Total frame size = 2 (SOF+LEN) + LEN
 *
 * Return sig:
 *   0x00 = OK
 *   0x01 = bad checksum
 *   0x02 = unknown class
 *   0x03 = unknown function
 *   0x04 = bad payload
 *   0x05 = sanity check failed. Either packet is too long or len<2 (min packet size)
 *   0x06 = Tx error, not everything was sent
 *
 *   0x10-0x1F => Lid Specific errors
 *   0x20-0x2F => Nob Specific errors
 """
*/
 #ifndef TCP_PROCESSOR_H
 #define TCP_PROCESSOR_H

 #include <Arduino.h>
 #include <WiFi.h>
 #include "MPU6500.h"
 #include "Stepper.h"
 #include <time.h>

 class TCPprocessor{
  public:
    TCPprocessor(IPAddress localIP, IPAddress gateway, IPAddress subnet, Stepper &stepper, int sampleRate, uint16_t port = 5010);

    void begin();

    uint8_t listen();
    uint8_t sendPacket(uint8_t len, uint8_t CID, uint8_t FID, uint8_t *packet);

    MPU6500 mpuLeg;
    MPU6500 mpuFoot;
    int sampleRate;
    
  private:
    uint8_t processPacket(uint8_t *packet, uint8_t len);
    void    makeFakeSample(uint8_t* out);

    uint8_t SOF = 0xAA;
    static constexpr uint8_t MAX_PKT  = 64;

    IPAddress   _localIP, _gateway, _subnet;
    const char* _ssid;
    const char* _pass;
    WiFiServer  _server;
    WiFiClient  _client;
    

    
    Stepper& stepperLeg; 

    uint8_t     _lenBuf;    // tmp byte for LEN
    
 };
 #endif