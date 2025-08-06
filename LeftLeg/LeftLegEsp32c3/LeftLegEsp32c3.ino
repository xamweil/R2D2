#include <WiFi.h>
#include "secrets.h"
#include "TCPprocessor.h"

IPAddress localIP(192, 168, 66, 10);
IPAddress gateway(192, 168, 66, 1);
IPAddress subnet(255, 255, 255, 0);

TCPprocessor* tcp;

uint8_t taster1 = D0;
uint8_t taster2 = D1;

uint8_t out[16]; // Buffer for sensor data

void setup() {
  // Start TCP connection
  tcp = new TCPprocessor(localIP, gateway, subnet);
  tcp->begin();
  pinMode(taster1, INPUT_PULLUP);
  pinMode(taster2, INPUT_PULLUP);
}

void loop(){
    // listen for incoming packets
    tcp->listen();
    // send sensor data
    tcp->mpuLeg.getPackedSample(out);
    tcp->sendPacket(16, 0x01, 0x02, out);
    tcp->mpuFoot.getPackedSample(out);
    tcp->sendPacket(16, 0x02, 0x02, out);
    // Taster signals to jetson
    if (digitalRead(taster1) == LOW) {
        tcp->sendPacket(0, 0x04, 0x01, nullptr);
    }
    else if (digitalRead(taster2) == LOW) {
        tcp->sendPacket(0, 0x04, 0x02, nullptr);
    }
    delay(10); 
}

    
