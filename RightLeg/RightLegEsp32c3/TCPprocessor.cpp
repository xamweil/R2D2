#include "TCPprocessor.h"
#include "secrets.h"
TCPprocessor::TCPprocessor(IPAddress localIP, IPAddress gateway, IPAddress subnet, Stepper &stepper, int sampleRate, uint16_t port):
    _localIP(localIP),
    _gateway(gateway), 
    _subnet(subnet),
    _ssid(WIFI_SSID), 
    _pass(WIFI_PASSWORD), 
    _server(port),
    sampleRate(sampleRate),
    mpuLeg(0x69),
    mpuFoot(0x68),
    stepperLeg(stepper)
    {}
        

void TCPprocessor::begin() {
        stepperLeg.setSpeed(10); // Set default speed for stepper

         // Start MPU6500 sensors
        if (!mpuLeg.begin()) {
            delay(1);
            // implement tcp sig to jetson

        }
        if (!mpuFoot.begin()) {
            delay(1);
            // implement tcp sig to jetson
        }

        // Set sample rate and ranges
        mpuLeg.setSampleRate(sampleRate); // Set sample rate here
        mpuLeg.setAccelRange(0);  // ±2g
        mpuLeg.setGyroRange(0);   // ±250°/s

        mpuFoot.setSampleRate(sampleRate); // Set sample rate here
        mpuFoot.setAccelRange(0);  // ±2g
        mpuFoot.setGyroRange(0);   // ±250°/s
    
        /*
        * Start TCP connection
        */
        WiFi.mode(WIFI_STA);
        WiFi.config(_localIP, _gateway, _subnet);
        WiFi.begin(_ssid, _pass);

        while (WiFi.status() != WL_CONNECTED) delay(100);
        _server.begin();


        sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);   // optional; slews tiny corrections
        sntp_set_sync_interval(60000);               // optional; resync every 60 s
        configTzTime("UTC22", "192.168.66.1");        // router IP; keep UTC (no DST math)

        struct tm tmNow;
        if (getLocalTime(&tmNow, 10000)) {           // wait up to 10 s once at boot
            Serial.printf("NTP UTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                        tmNow.tm_year + 1900, tmNow.tm_mon + 1, tmNow.tm_mday,
                        tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec);
        } else {
            Serial.println("NTP sync failed");
        }
    }

uint8_t TCPprocessor::listen(){
    if (!_client || !_client.connected()) {
        _client = _server.available();            // may be null
        if (!_client) return 0xFF;                // no peer yet
        _client.setTimeout(50);                   // 50 ms read timeout
    }

    // Look for SOF
    if (!_client.available()) return 0xFF;
    if (_client.peek() != SOF) {
        _client.read();                           // discard noise byte
        return 0xFF;
    }
    _client.read();                               // consume SOF

    // Read len
    if (!_client.readBytes(&_lenBuf, 1)) return 0xFF;   // timed out
    uint8_t len = _lenBuf;
    if (len < 2 || len > MAX_PKT) return 0x05;          // sanity check

    // Read rest of message
    uint8_t pkt[MAX_PKT];
    if (_client.readBytes(pkt, len) != len) return 0xFF; // timeout

    return processPacket(pkt, len);
}

uint8_t TCPprocessor::sendPacket(uint8_t payloadLen, uint8_t CID, uint8_t FID, uint8_t *packet) {
    if (!_client || !_client.connected()) return 0xFE;   // no peer
    const uint8_t LEN = payloadLen + 2; // CID + FID + payload
    const uint8_t total = 2 + LEN; // SOF + LEN + rest
    if (total > MAX_PKT) return 0x05;

    uint8_t frame[MAX_PKT];

    frame[0] = SOF;
    frame[1] = LEN;
    frame[2] = CID;
    frame[3] = FID;

    if (payloadLen) {
        if (!packet) return 0x04;                // bad payload
            memcpy(&frame[4], packet, payloadLen); // copy payload
    }
    
    size_t written = _client.write(frame, total);
    _client.flush();                        // push immediately

    return (written == total) ? 0x00 : 0x06;
}

uint8_t TCPprocessor::processPacket(uint8_t *packet, uint8_t len){
    uint8_t CID = packet[0];
    uint8_t FID = packet[1];
    uint8_t payloadlen = len -2;

    uint8_t resp = 0x00;
    const uint8_t* payload = &packet[2];

    MPU6500* sensor= nullptr;
    Stepper* stepper = nullptr;

    switch (CID)
    {
        case 0x01:
            sensor = &mpuLeg;
            break;
        case 0x02:
            sensor = &mpuFoot;
            break;
        case 0x03:
            stepper = &stepperLeg;
            break;    
        default:
            return 0x02; // unknown class

    }
    if(sensor){
        switch (FID)
        {
            case 0x01: {
                int16_t temp = sensor->readTemperature();
                sendPacket(2, CID, FID, (uint8_t*)&temp);
                return 0x00; // OK
            }           
            case 0x02: {
                uint8_t out[16];
                sensor->getPackedSample(out);
                sendPacket(16, CID, FID, out);
                return 0x00; // OK
            }
            default:
                return 0x03; // unknown function
        }
    }
    
    else if(stepper){
        switch (FID)
        {
            case 0x01:{// close position
                stepper->step(530);
                stepper->motorOff();
                sendPacket(1, CID, FID, &resp);
                return 0x00; // OK
            } 
                
            case 0x02: { // open position
                stepper->step(-512);
                stepper->motorOff();
                sendPacket(1, CID, FID, &resp);
                return 0x00; // OK
            }
            case 0x03: { // set speed
                stepper->setSpeed(payload[0]); // is save between 0 and 10
                sendPacket(1, CID, FID, &resp);
                return 0x00; // OK
            }
            case 0x04: { //move pos
                int16_t step = (int16_t)((payload[0] << 8) | payload[1]); 
                stepper->step(step); 
                sendPacket(1, CID, FID, &resp);
                return 0x00; // OK


            }
            default:
                resp = 0x03;
                sendPacket(1, CID, FID, &resp);
                return 0x03; // unknown function
        }
    }    
}

