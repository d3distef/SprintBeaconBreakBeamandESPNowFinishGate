/* 
  Sprint Beacon – ESP32-S3 Mini/Zero (FINISH GATE)
  - OTA only when GPIO12 held LOW ≥250 ms at boot
  - ESP-NOW peer to Start Gate (uses START_GATE_MAC)
  - BLE (NimBLE): status/range + sprint_ms (uint32 LE)
  - LiDAR (XT-S1) over UART1 Modbus
  - Added: ESP-NOW broadcast peer + RX watchdog re-init
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoOTA.h>
#include <NimBLEDevice.h>
#include "secrets.h"        // WIFI_SSID, WIFI_PASS

// ===== Config =====
static const char* OTA_HOSTNAME = "beacon-finish";
static const char* OTA_PASSWORD = "U";
#define DEBUG 0
#define FORCE_AP_RESCUE 0

// ===== GPIOs =====
static const uint8_t PIN_LED       = 7;
static const uint8_t PIN_LASER     = 8;
static const uint8_t PIN_IR_INPUT  = 13;
static const uint8_t PIN_OTA_BTN   = 12;   // momentary to GND

// ===== Timings (ms) =====
static const uint32_t BOOT_LASER_ON_DELAY_MS = 1000;
static const uint32_t INTACT_OFF_DELAY_MS    = 1000;
static const uint32_t BROKEN_ON_DELAY_MS     = 3000;

// ===== LiDAR / Modbus (XT-S1) =====
HardwareSerial LIDAR(1);
static const int LIDAR_BAUD   = 115200;
static const int LIDAR_RX_PIN = 44;
static const int LIDAR_TX_PIN = 43;
static const uint8_t  MB_DEV_ID     = 0x01;
static const uint32_t MB_POLL_MS    = 100;
static const uint32_t MB_RX_TIMEOUT = 80;

// ===== Runtime =====
static bool     laserOn = false;
static bool     prevBeamIntact = false;
static bool     manualOverride  = false;   // false=AUTO, true=MANUAL
static bool     manualState     = false;   // desired in MANUAL
static uint32_t tBoot = 0, tBeamIntactStart = 0, tBeamBrokenStart = 0;
static int32_t  lidarDistanceCm = -1;
static uint32_t bootCount = 0;
static char     lastCmd[40] = "-";
static bool     otaMode = false;
static uint8_t  myStaMac[6] = {0};         // filled when ESP-NOW starts

// --- From Start gate via ESP-NOW ---
static volatile uint8_t  startPeerBeam = 1;   // 1=INTACT, 0=BROKEN
static uint8_t           startPeerBeamPrev = 1;
static uint32_t          startBlinkUntil = 0;

// ===== Visual + counters for ANY RX =====
static uint32_t          rxTickUntil = 0;     // short LED tick on any ESP-NOW RX
static volatile uint32_t rx_count = 0;        // total packets received
static volatile uint32_t last_rx_any_ms = 0;  // last time any ESP-NOW packet arrived

// ===== ESP-NOW (Finish <-> Start) =====
static const uint8_t ESPNOW_CHANNEL = 1;    // must match Start Gate
uint8_t START_GATE_MAC[6] = { 0x50, 0x78, 0x7D, 0x18, 0x83, 0xB8 };  // <<< set to your Start Gate STA MAC
static const uint8_t BCAST[6]       = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF }; // broadcast peer

// 7-byte heartbeat from Start
typedef struct __attribute__((packed)) {
  uint32_t uptime_ms;
  uint8_t  beam_intact;   // 1=INTACT, 0=BROKEN
  int16_t  rssi;          // placeholder
} BeaconMsgWire7;

typedef struct __attribute__((packed)) {
  uint32_t uptime_ms;
  uint8_t  beam_intact;
  int16_t  rssi;
} BeaconMsg;

typedef struct __attribute__((packed)) {
  uint8_t  msg_type;      // 2 = START
  uint32_t start_ms;      // Start Gate's millis() at trigger
} StartMsg;

static_assert(sizeof(BeaconMsgWire7) == 7, "BeaconMsgWire7 must be 7 bytes");
static_assert(sizeof(BeaconMsg)      == 7, "BeaconMsg must be 7 bytes");
static_assert(sizeof(StartMsg)       == 5, "StartMsg must be 5 bytes");

static volatile uint32_t lastRxMs = 0;   // when we last heard a heartbeat (unicast)
static bool espNowStarted = false;

// ----- Start/Finish timing state -----
static bool     haveTimeOffset   = false;   // true once a heartbeat arrives
static int32_t  timeOffsetMs     = 0;       // local_ms ≈ remote_ms + timeOffsetMs
static bool     haveStart        = false;   // true after receiving StartMsg
static uint32_t remoteStartMs    = 0;       // Start Gate's millis() at trigger
static uint32_t startSeenLocalMs = 0;       // our local millis() when StartMsg arrived
static uint32_t lastFinishMs     = 0;       // our local millis() at finish beam break
static uint32_t lastSprintMs     = 0;       // computed sprint time (ms)
static const   uint32_t START_STALE_MS = 10000; // require finish within 10s of start

// ===== BLE (service + chars) =====
#define UUID_SERVICE        "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0a1"
#define UUID_CHAR_STATUS    "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0b2"  // READ | NOTIFY (JSON)
#define UUID_CHAR_CONTROL   "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0c3"  // WRITE (JSON cmds)
#define UUID_CHAR_RANGE     "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0d4"  // READ | NOTIFY (uint16 cm)
#define UUID_CHAR_SPRINT    "b8c7f3f4-4b9f-4a5b-9c39-36c6b4c7e0f6"  // READ | NOTIFY (uint32 ms)

static NimBLEServer*         bleServer   = nullptr;
static NimBLEService*        bleService  = nullptr;
static NimBLECharacteristic* chStatus    = nullptr;
static NimBLECharacteristic* chControl   = nullptr;
static NimBLECharacteristic* chRange     = nullptr;
static NimBLECharacteristic* chSprint    = nullptr;
static NimBLEAdvertising*    bleAdv      = nullptr;
static volatile bool         bleClientConnected = false;

// ===== Helpers =====
inline void setLaser(bool on){
  if (laserOn != on) { laserOn = on; digitalWrite(PIN_LASER, on ? HIGH : LOW); }
}

// --- Modbus helpers (needed by setup/loop) ---
static uint16_t mb_crc16(const uint8_t* d, size_t n){
  uint16_t c=0xFFFF;
  for(size_t i=0;i<n;i++){
    c ^= d[i];
    for(int b=0;b<8;b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}
static bool mb_txrx(const uint8_t* req, size_t rlen, uint8_t* rx, size_t& xlen, uint32_t to){
  while (LIDAR.available()) LIDAR.read();
  LIDAR.write(req, rlen); LIDAR.flush();
  uint32_t t0 = millis(); size_t i = 0;
  while (millis() - t0 < to) { while (LIDAR.available()) { if (i < xlen) rx[i++] = uint8_t(LIDAR.read()); } delay(1); }
  xlen = i; return i >= 5;
}
static bool mb_read_regs(uint8_t dev,uint8_t func,uint16_t reg,uint16_t cnt,uint8_t* data,uint8_t& nbytes){
  uint8_t req[8]={dev,func,uint8_t(reg>>8),uint8_t(reg),uint8_t(cnt>>8),uint8_t(cnt),0,0};
  uint16_t crc=mb_crc16(req,6); req[6]=crc&0xFF; req[7]=crc>>8;
  uint8_t rx[64]; size_t x=sizeof(rx);
  if(!mb_txrx(req,sizeof(req),rx,x,MB_RX_TIMEOUT)) return false;
  if(x<5) return false;
  uint16_t rxcrc=rx[x-2]|(rx[x-1]<<8);
  if(mb_crc16(rx,x-2)!=rxcrc) return false;
  if(rx[0]!=dev || rx[1]==(func|0x80) || rx[1]!=func) return false;
  uint8_t bc=rx[2]; if(bc!=cnt*2) return false;
  memcpy(data,rx+3,bc); nbytes=bc; return true;
}
static bool mb_write_u16(uint8_t dev,uint16_t reg,uint16_t val){
  uint8_t req[8]={dev,0x06,uint8_t(reg>>8),uint8_t(reg),uint8_t(val>>8),uint8_t(val),0,0};
  uint16_t crc=mb_crc16(req,6); req[6]=crc&0xFF; req[7]=crc>>8;
  uint8_t rx[16]; size_t x=sizeof(rx);
  if(!mb_txrx(req,sizeof(req),rx,x,MB_RX_TIMEOUT)) return false;
  if(x<8) return false;
  uint16_t rxcrc=rx[x-2]|(rx[x-1]<<8);
  return (mb_crc16(rx,x-2)==rxcrc && rx[0]==dev && rx[1]==0x06);
}

// ===== Wi-Fi + OTA helpers (needed by setup) =====
static bool gOTAStarted=false;

static void startOTAOnce(){
  if(gOTAStarted) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){ digitalWrite(PIN_LASER, LOW); });
  ArduinoOTA.begin();
  gOTAStarted=true;
}
static void startAPFallback(){
  String tail=String((uint32_t)ESP.getEfuseMac(),HEX).substring(4);
  String ssid=String("SprintBeacon-")+tail;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid.c_str(),"BeaconSetup");
  delay(150);
  startOTAOnce();
}
static void connectWiFiAndStartOTA(){
#if FORCE_AP_RESCUE
  startAPFallback(); return;
#endif
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<15000) delay(250);
  if(WiFi.status()==WL_CONNECTED){ startOTAOnce(); }
  else { startAPFallback(); }
}

// ===== ESP-NOW callbacks =====
void onEspNowSent(const wifi_tx_info_t* info, esp_now_send_status_t status){
  (void)info; (void)status;
}

void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  uint32_t nowLocal = millis();

  // ANY RX — tick + counters
  rx_count++;
  last_rx_any_ms = nowLocal;
  rxTickUntil = nowLocal + 120;

#if DEBUG
  if (info) {
    const uint8_t* smac = info->src_addr;
    Serial.printf("[NOW][RX] len=%d from %02X:%02X:%02X:%02X:%02X:%02X ch=%d\n",
      len, smac[0],smac[1],smac[2],smac[3],smac[4],smac[5], (int)info->rx_ctrl.channel);
  }
#endif

  // 7-byte heartbeat
  if (len == 7) {
    BeaconMsgWire7 ob; memcpy(&ob, data, sizeof(ob));
    lastRxMs       = nowLocal;                                // last heartbeat time
    timeOffsetMs   = (int32_t)nowLocal - (int32_t)ob.uptime_ms;
    haveTimeOffset = true;
    startPeerBeam  = ob.beam_intact ? 1 : 0;
    return;
  }

  if (len == (int)sizeof(BeaconMsg)) {
    BeaconMsg m; memcpy(&m, data, sizeof(m));
    lastRxMs       = nowLocal;
    timeOffsetMs   = (int32_t)nowLocal - (int32_t)m.uptime_ms;
    haveTimeOffset = true;
    startPeerBeam  = m.beam_intact ? 1 : 0;
    return;
  }

  if (len == 5) {
    StartMsg s; memcpy(&s, data, sizeof(s));
    if (s.msg_type == 2) {
      remoteStartMs    = s.start_ms;
      haveStart        = true;
      startSeenLocalMs = nowLocal;
      startBlinkUntil  = nowLocal + 400;   // visible cue on START
    }
    return;
  }
}

// Bring up ESP-NOW on fixed channel and add peers
bool startEspNow(uint8_t channel){
  // Bring up Wi-Fi driver in STA, no association
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44));

  // Country 1–11
  wifi_country_t c = { "US", 1, 11, WIFI_COUNTRY_POLICY_MANUAL };
  esp_wifi_set_country(&c);

  // Force channel BEFORE esp_now_init()
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_promiscuous(false);

  // Read and cache our STA MAC (used for BLE adverts)
  uint8_t mac_sta[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac_sta);
  memcpy(myStaMac, mac_sta, 6);

  esp_err_t e = esp_now_init();
  if (e != ESP_OK) {
#if DEBUG
    Serial.printf("[NOW] esp_now_init failed: %d\n", (int)e);
#endif
    return false;
  }

  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  // Add Start (unicast) peer — ok if it fails; RX still works
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, START_GATE_MAC, 6);
  peer.channel = channel; peer.ifidx = WIFI_IF_STA; peer.encrypt = false;
  esp_err_t e1 = esp_now_add_peer(&peer);
#if DEBUG
  if (e1 != ESP_OK) Serial.printf("[NOW] add_peer(Start) failed: %d\n", (int)e1);
#endif

  // Add broadcast peer so we reliably see 0xA5 + broadcast heartbeats
  esp_now_peer_info_t bpeer{}; memcpy(bpeer.peer_addr, BCAST, 6);
  bpeer.channel = channel; bpeer.ifidx = WIFI_IF_STA; bpeer.encrypt = false;
  esp_err_t e2 = esp_now_add_peer(&bpeer);
#if DEBUG
  if (e2 != ESP_OK) Serial.printf("[NOW] add_peer(BCAST) failed: %d\n", (int)e2);
#endif

  espNowStarted = true;
  return true;
}

// ===== BLE (adds sprint_ms characteristic) =====
class SvrCB : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* s) { (void)s; bleClientConnected = true; }
  void onConnect(NimBLEServer* s, ble_gap_conn_desc*) { onConnect(s); }
  void onDisconnect(NimBLEServer* s) { bleClientConnected = false; if (s) s->startAdvertising(); }
};
class CtrlCB : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic* c) {
    std::string v = c->getValue();
    strncpy(lastCmd, v.c_str(), sizeof(lastCmd)-1);
    lastCmd[sizeof(lastCmd)-1] = '\0';
    if (v.find("\"laser\"") != std::string::npos) {
      bool val = (v.find(":1") != std::string::npos) || (v.find("true") != std::string::npos);
      manualOverride = true; manualState = val; setLaser(val);
    }
    if (v.find("\"auto\"") != std::string::npos) {
      bool val = (v.find(":1") != std::string::npos) || (v.find("true") != std::string::npos);
      manualOverride = !val;
    }
  }
};

// Manufacturer data: 0xA1 + 6-byte STA MAC + uint16 range (LE)
static void setAdvWithMacAndRange(uint16_t cmLE){
  NimBLEAdvertisementData advData, scanData;

  // Device name with MAC tail for human scanning
  char name[32];
  snprintf(name, sizeof(name), "SprintFinish-%02X%02X",
           myStaMac[4], myStaMac[5]);     // short tail in the name

  advData.setCompleteServices(NimBLEUUID(UUID_SERVICE));

  uint8_t mfg[1 + 6 + 2];
  mfg[0] = 0xA1;                                   // tag
  memcpy(mfg + 1, myStaMac, 6);                    // STA MAC
  mfg[7] = uint8_t(cmLE & 0xFF);                   // range L
  mfg[8] = uint8_t((cmLE >> 8) & 0xFF);            // range H
  advData.setManufacturerData(std::string((char*)mfg, sizeof(mfg)));

  scanData.setName(name);

  bleAdv->setAdvertisementData(advData);
  bleAdv->setScanResponseData(scanData);
}

static bool startBLE(){
  // Build name with MAC tail (if we already have it; else generic)
  char devName[32];
  if (myStaMac[0]|myStaMac[1]|myStaMac[2]|myStaMac[3]|myStaMac[4]|myStaMac[5]) {
    snprintf(devName, sizeof(devName), "SprintFinish-%02X%02X", myStaMac[4], myStaMac[5]);
  } else {
    snprintf(devName, sizeof(devName), "SprintFinish");
  }

  NimBLEDevice::init(devName);
  NimBLEDevice::setDeviceName(devName);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new SvrCB());

  bleService = bleServer->createService(UUID_SERVICE);
  chStatus  = bleService->createCharacteristic(UUID_CHAR_STATUS,  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chControl = bleService->createCharacteristic(UUID_CHAR_CONTROL, NIMBLE_PROPERTY::WRITE);
  chRange   = bleService->createCharacteristic(UUID_CHAR_RANGE,   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chSprint  = bleService->createCharacteristic(UUID_CHAR_SPRINT,  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  chControl->setCallbacks(new CtrlCB());

  chStatus->setValue("{\"boot\":true}");
  uint16_t seed=0xFFFF; chRange->setValue((uint8_t*)&seed, 2);
  uint32_t zero=0;       chSprint->setValue((uint8_t*)&zero, 4);

  bleService->start();
  bleAdv = NimBLEDevice::getAdvertising();

  // Advert now contains full STA MAC (in mfg data) + MAC tail in the name
  setAdvWithMacAndRange(0xFFFF);
  bleAdv->start();
  return true;
}

// ===== Setup / Loop =====
void setup() {
  Serial.begin(115200);
  delay(50);
  bootCount++;

  // Decide OTA at boot (GPIO12 LOW for ≥250 ms)
  pinMode(PIN_OTA_BTN, INPUT_PULLUP);
  bool bootOta = false;
  int64_t us0 = esp_timer_get_time();
  while ((esp_timer_get_time() - us0) < 250000) {      // 250 ms
    if (digitalRead(PIN_OTA_BTN) == HIGH) break;       // released early
  }
  if (digitalRead(PIN_OTA_BTN) == LOW) bootOta = true;

  pinMode(PIN_LED, OUTPUT);   digitalWrite(PIN_LED, LOW);
  pinMode(PIN_LASER, OUTPUT); digitalWrite(PIN_LASER, LOW);
  pinMode(PIN_IR_INPUT, INPUT_PULLUP);

  // LiDAR UART (ok if nothing attached)
  LIDAR.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  mb_write_u16(MB_DEV_ID, 0x003D, 0x0001);
  mb_write_u16(MB_DEV_ID, 0x0042, 0x0032);
  mb_write_u16(MB_DEV_ID, 0x0003, 0x0001);

  // Base beam state
  tBoot = millis();
  bool beamIntact = (digitalRead(PIN_IR_INPUT)==HIGH);
  prevBeamIntact = beamIntact;
  if (beamIntact) { tBeamIntactStart=tBoot; tBeamBrokenStart=0; }
  else            { tBeamBrokenStart=tBoot; tBeamIntactStart=0; }

  if (bootOta) {
    connectWiFiAndStartOTA();
    // myStaMac remains 0s in AP/OTA; BLE will still start with generic name
    startBLE();
  } else {
    if (!startEspNow(ESPNOW_CHANNEL)) {
      connectWiFiAndStartOTA(); // rescue path
    }
    // By now myStaMac is set (startEspNow did it). Start BLE with MAC in adverts.
    startBLE();
  }

  // LED mirrors beam initially
  digitalWrite(PIN_LED, beamIntact ? HIGH : LOW);
}

void loop() {
  ArduinoOTA.handle();
  uint32_t now = millis();

  // —— ESPNOW re-arm if the RF path looks dead (no ANY RX for 5s) —
  static uint32_t lastRearm=0;
  if (espNowStarted && (now - last_rx_any_ms > 5000) && (now - lastRearm > 6000)) {
#if DEBUG
    Serial.println("[NOW] No RX for 5s → reinit ESPNOW");
#endif
    esp_now_deinit(); espNowStarted=false;
    startEspNow(ESPNOW_CHANNEL);
    lastRearm = now;
  }

  // Expire stale START if too old
  if (haveStart && (now - startSeenLocalMs) > START_STALE_MS) {
    haveStart = false;
  }

  // Beam + LED mirror + blinks
  bool beamIntact = (digitalRead(PIN_IR_INPUT)==HIGH);

  // Detect Start-gate INTACT -> BROKEN edge from heartbeat
  if (startPeerBeamPrev == 1 && startPeerBeam == 0) {
    startBlinkUntil = now + 400;            // blink for 0.4s
  }
  startPeerBeamPrev = startPeerBeam;

  // LED priorities:
  if (now < startBlinkUntil) {
    digitalWrite(PIN_LED, ((now / 100) % 2) ? HIGH : LOW);   // ~5 Hz toggle
  } else if (now < rxTickUntil) {
    digitalWrite(PIN_LED, ((now / 60) % 2) ? HIGH : LOW);    // quick tick on any RX
  } else {
    digitalWrite(PIN_LED, beamIntact ? HIGH : LOW);
  }

  if (beamIntact != prevBeamIntact) {
    if (beamIntact) {
      tBeamIntactStart = now; 
      tBeamBrokenStart = 0;
    } else {
      tBeamBrokenStart = now; 
      tBeamIntactStart = 0;

      // Finish trigger: compute sprint only if we have a recent START and a valid offset
      if (haveStart && haveTimeOffset) {
        lastFinishMs = now;
        uint32_t estLocalStart = (uint32_t)((int32_t)remoteStartMs + timeOffsetMs);
        if ((int32_t)(lastFinishMs - estLocalStart) >= 0) {
          lastSprintMs = lastFinishMs - estLocalStart;
        } else {
          lastSprintMs = 0;
        }

        // Update sprint characteristic (uint32 LE) and notify
        if (chSprint) {
          chSprint->setValue((uint8_t*)&lastSprintMs, 4);
          if (bleClientConnected) chSprint->notify();
        }

        haveStart = false;  // require a new START for the next sprint
      }
    }
    prevBeamIntact = beamIntact;
  }

  // Laser policy unless manual override
  if (!manualOverride) {
    bool wantLaser = (now - tBoot >= BOOT_LASER_ON_DELAY_MS);
    if (beamIntact && tBeamIntactStart && (now - tBeamIntactStart >= INTACT_OFF_DELAY_MS)) wantLaser=false;
    else if (!beamIntact && tBeamBrokenStart && (now - tBeamBrokenStart > BROKEN_ON_DELAY_MS)) wantLaser=true;
    setLaser(wantLaser);
  } else {
    setLaser(manualState);
  }

  // LiDAR poll (→ centimeters)
  static uint32_t lastPoll = 0;
  if (now - lastPoll >= MB_POLL_MS) {
    lastPoll = now;
    bool updated = false;
    { uint8_t d[6]; uint8_t nb = sizeof(d);
      if (mb_read_regs(MB_DEV_ID, 0x04, 0x0017, 3, d, nb)) {
        uint16_t raw_mm = (d[0] << 8) | d[1];
        if (raw_mm < 0xFFF0){ lidarDistanceCm = (int32_t)((raw_mm + 5) / 10); updated = true; }
      }
    }
    if(!updated){
      uint8_t c[2]; uint8_t nb = sizeof(c);
      if (mb_read_regs(MB_DEV_ID,0x04,0x002E,1,c,nb)){
        uint16_t raw_cm = (c[0]<<8) | c[1];
        if (raw_cm < 0xFFF0) lidarDistanceCm = (int32_t)raw_cm;
      }
    }
  }

  // ===== BLE updates (status + range periodic) =====
  static uint32_t lastBle=0, lastRangeNotify=0, lastAdv=0;
  if (now - lastBle >= 300){
    lastBle = now;
    const IPAddress ip = (WiFi.getMode()==WIFI_MODE_AP) ? WiFi.softAPIP() : WiFi.localIP();
    float sprint_s = (lastSprintMs > 0) ? (lastSprintMs / 1000.0f) : 0.0f;

    uint32_t start_age = haveStart ? (now - startSeenLocalMs) : 0;
    uint32_t link_age  = lastRxMs ? (now - lastRxMs) : 0;
    uint32_t link_any  = last_rx_any_ms ? (now - last_rx_any_ms) : 0;

    char buf[520];
    snprintf(buf,sizeof(buf),
      "{\"beam\":%s,\"laser\":%s,\"lidar_cm\":%ld,\"uptime\":%lu,"
      "\"auto\":%s,\"ip\":\"%u.%u.%u.%u\",\"boot\":%u,\"last\":\"%s\","
      "\"have_start\":%s,\"sprint_ms\":%lu,\"sprint_s\":%.3f,"
      "\"peer_beam\":%u,\"start_age_ms\":%lu,\"link_ms\":%lu,"
      "\"rx_count\":%lu,\"link_ms_any\":%lu,"
      "\"sta_mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}",
      beamIntact ? "true":"false",
      laserOn ? "true":"false",
      (long)lidarDistanceCm,
      (unsigned long)now,
      manualOverride ? "false":"true",
      ip[0],ip[1],ip[2],ip[3],
      bootCount, lastCmd,
      haveStart ? "true":"false",
      (unsigned long)lastSprintMs,
      sprint_s,
      (unsigned)startPeerBeam,
      (unsigned long)start_age,
      (unsigned long)link_age,
      (unsigned long)rx_count,
      (unsigned long)link_any,
      myStaMac[0],myStaMac[1],myStaMac[2],myStaMac[3],myStaMac[4],myStaMac[5]);

    if (chStatus){ chStatus->setValue((uint8_t*)buf, strlen(buf)); if (bleClientConnected) chStatus->notify(); }
  }

  if (now - lastRangeNotify >= 300){
    lastRangeNotify = now;
    uint16_t ucm = (lidarDistanceCm >= 0 && lidarDistanceCm <= 65534) ? (uint16_t)lidarDistanceCm : (uint16_t)0xFFFF;
    if (chRange){
      chRange->setValue((uint8_t*)&ucm, 2);
      if (bleClientConnected) chRange->notify();
    }
  }

  if (!bleClientConnected && now - lastAdv >= 800){
    lastAdv = now;
    uint16_t ucm = (lidarDistanceCm >= 0 && lidarDistanceCm <= 65534) ? (uint16_t)lidarDistanceCm : (uint16_t)0xFFFF;
    setAdvWithMacAndRange(ucm);  // keep MAC in adverts fresh alongside range
  }

  delay(2);
}
