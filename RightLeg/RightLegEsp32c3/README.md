
---

# RightLegEsp32c3 – ESP32-C3 IMU + Stepper + Taster (TCP)

Firmware for a **Seeed XIAO ESP32-C3** driving one stepper, streaming two **MPU-6500** IMUs (leg & foot), and reporting two “taster” buttons over a tiny TCP protocol to the Jetson.

* **This folder = rigth leg** (default `192.168.66.11:5011`).
* Left leg, uses the same firmware and only has different IP/port.

## What’s here

* `RigthLegEsp32c3.ino` – main sketch (sets IP/port, pins, creates `TCPprocessor`).&#x20;
* `TCPprocessor.h/.cpp` – Wi-Fi + TCP server, message framing, SNTP time init.
* `MPU6500.h/.cpp` – minimal IMU driver (no FIFO), DATA\_RDY-gated reads, 16-byte pack.&#x20;
* `Stepper.h/.cpp` – simple stepper helper (pins set in the sketch).&#x20;
* `secrets_template.h` – copy to `secrets.h`, fill WiFi; `secrets.h` is gitignored.&#x20;

## Hardware

* Seeed **XIAO ESP32-C3**
* Two **MPU-6500** on I²C (`0x69` for leg, `0x68` for foot)
* Stepper motor + driver (pins defined in the sketch / stepper ctor)&#x20;
* Two buttons (“taster”) on **D0** and **D1** (internal pull-ups enabled in the sketch)
* GL-SFT1200 router (LAN `192.168.66.0/24`) providing NTP (router IP `192.168.66.1`)

> Note: I²C is kept at **100 kHz** for long internal runs; set in the IMU driver.&#x20;

## Build & Flash

1. Install Arduino IDE with Seeed **XIAO ESP32-C3** board support.
2. Copy secrets and edit:

   ```bash
   cp secrets_template.h secrets.h
   # edit secrets.h → WIFI_SSID / WIFI_PASSWORD
   ```


3. Open `RigthLegEsp32c3.ino` and confirm:

   * **IP/port** for the rigth leg: `192.168.66.11`, port `5011`.
   * **Gateway/Subnet** match your router.
4. Select **XIAO ESP32-C3** and upload.

## Network & Time

On boot the ESP joins Wi-Fi (static IP), starts a TCP server, and syncs time via **SNTP** against the router:

```cpp
// in TCPprocessor::begin()
configTime(/*gmtOffset*/3600, /*dst*/3600, "192.168.66.1");
if (getLocalTime(&tmNow, 10000)) { /* prints UTC time */ }
```



Timestamps placed into IMU packets are **UTC milliseconds** (u32), produced from the system clock.

## IMU sampling

* **Sample rate** is set from the sketch via `TCPprocessor(sampleRate=50)`, and forwarded to both IMUs: `setSampleRate(sampleRate)`.&#x20;
* Driver uses **no FIFO** and only reads when **DATA\_RDY** is set, so you actually stream at the configured rate. Packed payload = `ax, ay, az, gx, gy, gz, ts_ms (u32)` in **little endian**.&#x20;

## TCP frame format

```
SOF  LEN  CID  FID  PAYLOAD...
0xAA  N   1B   1B   LEN-2 bytes
```

* CIDs: `0x01` = leg IMU, `0x02` = foot IMU, `0x03` = motor, `0x04` = taster press signals.
* Return/status codes & sanity checks are summarized in `TCPprocessor.h`.&#x20;

## Buttons (“Tasters”)

* Pins `D0` and `D1` with internal pull-ups; when pressed, unsolicited frames with CID `0x04`, FID `0x01` or `0x02` are sent to the Jetson. Logic lives in `TCPprocessor`.&#x20;

## Stepper

* Created in the main sketch with your pins and steps-per-rev, then passed into `TCPprocessor` which exposes simple motor commands via the TCP protocol. See `Stepper.h` for the API.&#x20;

## Configure (quick checklist)

* `secrets.h` → **Wi-Fi SSID/PASSWORD**.&#x20;
* `RigthLegEsp32c3.ino` → **local IP/port/gateway/subnet**, **stepper pins**, **sample rate**.&#x20;
* Router (GL-SFT1200) → **Enable NTP server** on LAN (`192.168.66.1`).

## Troubleshooting

* **No time / “NTP sync failed”:** enable “Provide NTP server” on the router, or point to another reachable NTP server. SNTP is called once at boot in `TCPprocessor::begin()`.&#x20;
* **Sample rate higher than expected:** ensure you’re on the **no-FIFO** driver (this repo) where `update()` gates reads on DATA\_RDY and the configured interval.&#x20;
* **No packets:** confirm static IP is reachable and port matches (`5011`).

---


**Board:** Seeed XIAO ESP32-C3
**Sensors:** MPU-9250/6500

---
