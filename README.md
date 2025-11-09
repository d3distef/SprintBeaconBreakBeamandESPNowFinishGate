# SprintBeacon – Finish Gate (ESP32-S3 Mini/Zero)

**Finish gate** for the SprintBeacon system.  
Receives **START** from the Start Gate over **ESP-NOW**, detects **finish beam** locally, computes **sprint time**, and exposes live data via **BLE (NimBLE)**. Optional **OTA** is available on demand.

- Radio: **ESP-NOW** (fixed channel) + optional **BLE** status/control
- LiDAR: **XT-S1** over **UART1 Modbus**
- Visuals: on-board LED mirrors beam state + blinks on events
- Robustness: broadcast peer, RX watchdog re-init, AP fallback for OTA

---

## Features

- **ESP-NOW** unicast with the Start Gate (channel must match)
- **Heartbeat handling** (7-byte frames) to track link + beam state
- **START capture** → compute **sprint_ms** and notify over BLE
- **BLE (NimBLE)**:
  - Service: status JSON, control (manual/auto laser), range (cm), `sprint_ms` (uint32 LE)
  - Advertising includes **STA MAC** and **range** in mfg data
- **OTA on demand**: hold the **OTA button** at boot (GPIO12 LOW ≥ 250 ms)
- **RX watchdog**: if *no* ESP-NOW RX for 5 s, re-init the ESP-NOW stack
- **AP rescue**: if STA Wi-Fi fails, spawns `SprintBeacon-<MACTail>` AP for OTA

---

## Hardware

- **MCU**: ESP32-S3 Mini/Zero (Arduino core)
- **LiDAR**: XT-S1 (Modbus on UART1 @ 115200)
- **IR beam** sensor for finish detect (idle HIGH, broken LOW via `INPUT_PULLUP`)
- **Laser enable** output (drives your beam emitter transistor or module)
- **Power**: 5 V supply (USB-C breakout OK)

> If you later add an external WS2812 status LED, keep best practices:
> 330 Ω in series on DIN, ≥1000 µF cap across LED 5 V/GND, common ground.

---

## Pins (GPIO numbers)

```c
// Visual LED mirrors beam state and events
static const uint8_t PIN_LED       = 7;

// Laser output (drives HIGH to enable your laser/beam module)
static const uint8_t PIN_LASER     = 8;

// IR beam input — uses INPUT_PULLUP; expected idle HIGH, broken = LOW
static const uint8_t PIN_IR_INPUT  = 13;

// Hold LOW ≥ 250 ms at boot to enter OTA (STA or AP rescue)
static const uint8_t PIN_OTA_BTN   = 12;

// LiDAR (XT-S1) on UART1 / Modbus
static const int LIDAR_RX_PIN = 44;  // MCU RX
static const int LIDAR_TX_PIN = 43;  // MCU TX
static const int LIDAR_BAUD   = 115200;

IR sensor OUT ──> GPIO13     (other side of switch to GND; sensor should idle HIGH)
Laser driver    ──> GPIO8     (to transistor or module enable)
On-board LED     (GPIO7 toggled by firmware)

XT-S1 LiDAR     RX ─> GPIO44, TX ─> GPIO43, GND ↔ GND
5 V supply      ──> MCU 5V/VBUS
GND             ──> Common ground everywhere

OTA button      ──> GPIO12 ↔ GND   (hold LOW at boot for OTA)

