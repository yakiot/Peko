#include <Arduino.h>

#include <ESP32-TWAI-CAN.hpp>
#include "constants.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>

#if defined(PROTOCOL_J1939) == defined(PROTOCOL_OBD2)
#error "Select exactly one protocol: PROTOCOL_J1939 or PROTOCOL_OBD2"
#endif

static constexpr char SERVICE_UUID[] = "7e140001-8c55-4a82-b345-51fa952a6a01";
static constexpr char TELEMETRY_UUID[] = "7e140002-8c55-4a82-b345-51fa952a6a01";
static BLECharacteristic* telemetry = nullptr;
static std::atomic<bool> bleConnected{false};
static std::atomic<bool> restartAdvertising{false};
static uint32_t lastRpmNotify = 0;
static uint32_t lastSpeedNotify = 0;
static uint32_t lastFuelNotify = 0;
static uint32_t lastOdoNotify = 0;
static uint32_t lastLoadNotify = 0;
static uint32_t lastKeyNotify = 0;
static uint32_t lastThrottleNotify = 0;
static uint32_t lastPedalNotify = 0;
static char vehicleVin[18] = {};
static uint32_t lastVinNotify = 0;

// Validates a 17-byte VIN candidate (allowed charset only) and stores it in vehicleVin.
static void acceptVin(const uint8_t* data, size_t length) {
  if (length < 17) return;
  for (size_t i = 0; i < 17; ++i) {
    const char c = data[i];
    if (!((c >= 'A' && c <= 'Z' && c != 'I' && c != 'O' && c != 'Q') ||
          (c >= '0' && c <= '9'))) return;
  }
  memcpy(vehicleVin, data, 17);
  vehicleVin[17] = '\0';
}

// Republishes the cached VIN as "v=<VIN>" every 5 s so reconnecting clients receive it.
static void publishVin() {
  if (!vehicleVin[0] || !bleConnected.load() || millis() - lastVinNotify < 5000) return;
  lastVinNotify = millis();
  char payload[21];
  snprintf(payload, sizeof(payload), "v=%s", vehicleVin);
  telemetry->setValue(payload);
  telemetry->notify();
}

// GATT client connect/disconnect callbacks; flags are atomics consumed by the main loop.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected.store(true);
    Serial.println("BLE connected");
  }

  void onDisconnect(BLEServer*) override {
    bleConnected.store(false);
    restartAdvertising.store(true);
    Serial.println("BLE disconnected");
  }
};

static ServerCallbacks serverCallbacks;

// Initializes the "Yak-ELD" GATT server: one service, one read/notify telemetry
// characteristic with a CCCD, then starts advertising and logs the BLE address/UUIDs.
static void startBle() {
  BLEDevice::init("Yak-ELD");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(&serverCallbacks);
  BLEService* service = server->createService(SERVICE_UUID);
  telemetry = service->createCharacteristic(
      TELEMETRY_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  telemetry->addDescriptor(new BLE2902());
  telemetry->setValue("ready");
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();
  Serial.printf("BLE address: %s\n", BLEDevice::getAddress().toString().c_str());
  Serial.printf("BLE service: %s\nBLE telemetry: %s\n", SERVICE_UUID, TELEMETRY_UUID);
}

// Publishes "name=value" as a BLE notification, throttled to one per 200 ms per metric.
static void publishMetric(const char* name, float value, uint32_t& lastNotify) {
  if (!bleConnected.load() || millis() - lastNotify < 200) return;
  lastNotify = millis();
  char payload[20];
  snprintf(payload, sizeof(payload), "%s=%.1f", name, value);
  telemetry->setValue(payload);
  telemetry->notify();
}

// TWAI RX Pins
#define CAN_TX 5
#define CAN_RX 4

// ATGM336H-5N GPS on UART2: RX2 = GPIO16, TX2 = GPIO17, 9600 baud NMEA
#define GPS_RX 16
#define GPS_TX 17

CanFrame rxFrame;
static bool canReady = false;

static char nmeaLine[96];
static size_t nmeaLen = 0;
static uint32_t lastGpsNotify = 0;
static uint32_t nmeaSentences = 0;
static uint32_t nmeaRmcInvalid = 0;
static uint32_t lastGpsByteAt = 0;
static uint32_t lastGpsFixAt = 0;
static uint32_t lastGpsStatusLog = 0;

// Converts NMEA ddmm.mmmm / dddmm.mmmm to decimal degrees; S/W hemispheres are negative.
// Sends a CASIC config command to the ATGM336H: "$<body>*<xor-checksum>\r\n".
// Requires the module RX line wired to TX2 (GPIO17).
static void sendGpsCommand(const char* body) {
  uint8_t checksum = 0;
  for (const char* p = body; *p; ++p) checksum ^= uint8_t(*p);
  Serial2.printf("$%s*%02X\r\n", body, checksum);
}

static float nmeaToDegrees(const char* raw, char hemi) {
  if (!raw[0]) return 0.0f;
  const float v = atof(raw);
  const int deg = int(v / 100);
  float out = deg + (v - deg * 100) / 60.0f;
  if (hemi == 'S' || hemi == 'W') out = -out;
  return out;
}

// Parses one complete NMEA sentence. Only $GxRMC is used: requires status 'A' (valid fix),
// logs the position and publishes "g=<lat>,<lon>" throttled to 1 Hz.
static void handleNmea(char* s) {
  const size_t n = strlen(s);
  if (n < 20 || s[0] != '$' || strncmp(s + 3, "RMC", 3) != 0) return;
  char* f[12] = {};
  int count = 0;
  for (char* t = strtok(s, ","); t && count < 12; t = strtok(nullptr, ",")) f[count++] = t;
  if (count < 7) return;
  if (f[2][0] != 'A') {
    nmeaRmcInvalid++;
    if (GPS_DEBUG) Serial.println("   [GPS] RMC received but fix invalid (no satellite lock)");
    return;
  }
  const float lat = nmeaToDegrees(f[3], f[4][0]);
  const float lon = nmeaToDegrees(f[5], f[6][0]);
  if (lat == 0.0f && lon == 0.0f) {
    nmeaRmcInvalid++;
    if (GPS_DEBUG) Serial.println("   [GPS] RMC has empty coordinates");
    return;
  }
  lastGpsFixAt = millis();
  Serial.printf("   [GPS] %.5f, %.5f\n", lat, lon);
  if (!bleConnected.load() || millis() - lastGpsNotify < 1000) return;
  lastGpsNotify = millis();
  char payload[20];
  snprintf(payload, sizeof(payload), "g=%.4f,%.4f", lat, lon);
  telemetry->setValue(payload);
  telemetry->notify();
}

// Drains the UART2 RX buffer into nmeaLine; complete lines go to handleNmea().
// Called once per loop tick, so the UART buffer must hold a full second of NMEA.
static void pollGps() {
  while (Serial2.available()) {
    const char c = (char)Serial2.read();
    lastGpsByteAt = millis();
    if (c == '\n') {
      nmeaLine[nmeaLen] = '\0';
      nmeaSentences++;
      handleNmea(nmeaLine);
      nmeaLen = 0;
    } else if (c != '\r') {
      if (nmeaLen < sizeof(nmeaLine) - 1) nmeaLine[nmeaLen++] = c;
      else nmeaLen = 0;
    }
  }
  if (!GPS_DEBUG) return;
  const uint32_t now = millis();
  if (now - lastGpsStatusLog < 10000) return;
  lastGpsStatusLog = now;
  if (!lastGpsByteAt) {
    Serial.println("   [GPS] no UART data — check wiring (RX2=GPIO16 to module TX)");
  } else if (!lastGpsFixAt) {
    Serial.printf("   [GPS] %lu sentences, no valid fix yet (module alive, acquiring satellites?)\n",
                  (unsigned long)nmeaSentences);
  } else if (now - lastGpsFixAt > 10000) {
    Serial.printf("   [GPS] last fix %lu s ago\n", (unsigned long)((now - lastGpsFixAt) / 1000));
  }
}

#ifdef PROTOCOL_OBD2
static uint16_t obdEcu = 0;
static uint8_t pendingMode = 0;
static uint8_t pendingPid = 0;
static uint32_t requestSentAt = 0;
static uint32_t lastObdRequest = 0;
static uint32_t lastRpmRequest = 0;
static uint32_t lastSpeedRequest = 0;
static uint32_t lastAuxRequest = 0;
static bool supported[256] = {};
static int supportPage = 0;
static bool vinSupportChecked = false;
static bool vinSupported = false;
static uint8_t auxSlot = 0;
static uint8_t isoBuffer[32] = {};
static size_t isoLength = 0;
static size_t isoReceived = 0;
static uint8_t isoSequence = 1;
static uint32_t isoAt = 0;

// Decodes a big-endian uint32 from 4 bytes (OBD-II multi-byte fields).
static uint32_t bigEndian32(const uint8_t* bytes) {
  return (uint32_t(bytes[0]) << 24) | (uint32_t(bytes[1]) << 16) |
         (uint32_t(bytes[2]) << 8) | bytes[3];
}

// Handles a complete OBD-II response payload: supported-PID discovery, VIN assembly
// result, and Mode 01 metrics (RPM, speed, throttle, fuel, odometer) -> BLE.
static void finishObd(const uint8_t* data, size_t length, uint16_t source) {
  if (length >= 3 && data[0] == 0x7F && data[1] == pendingMode) {
    if (CAN_DEBUG) Serial.printf("OBD-II negative response mode=%02X code=%02X\n", pendingMode, data[2]);
    return;
  }
  if (length < 2 || data[0] != pendingMode + 0x40 || data[1] != pendingPid) return;
  if (pendingMode == 1 && supportPage >= 0 && pendingPid == supportPage) {
    if (length != 6) return;
    if (!obdEcu) obdEcu = source;
    const uint32_t mask = bigEndian32(data + 2);
    for (int bit = 0; bit < 32 && supportPage + bit + 1 < 256; ++bit) {
      supported[supportPage + bit + 1] = (mask & (uint32_t(1) << (31 - bit))) != 0;
    }
    supportPage = (mask & 1) && supportPage < 0xA0 ? supportPage + 0x20 : -1;
  } else if (pendingMode == 9 && pendingPid == 0) {
    if (length != 6) return;
    vinSupported = (bigEndian32(data + 2) & 0x40000000UL) != 0;
    vinSupportChecked = true;
  } else if (pendingMode == 9 && pendingPid == 2) {
    if (length != 20 || data[2] != 1) return;
    acceptVin(data + 3, 17);
  } else if (pendingMode == 1) {
    if (pendingPid == 0x0C && length == 4) {
      publishMetric("rpm", ((uint16_t(data[2]) << 8) | data[3]) / 4.0f, lastRpmNotify);
    } else if (pendingPid == 0x0D && length == 3) {
      publishMetric("speed", data[2], lastSpeedNotify);
    } else if (pendingPid == 0x11 && length == 3) {
      publishMetric("thr", data[2] * 100.0f / 255, lastThrottleNotify);
    } else if (pendingPid == 0x2F && length == 3) {
      publishMetric("fuel", data[2] * 100.0f / 255, lastFuelNotify);
    } else if (pendingPid == 0xA6 && length == 6) {
      publishMetric("odo", bigEndian32(data + 2) * 0.1f, lastOdoNotify);
    } else return;
  }
  pendingMode = 0;
  isoLength = 0;
}

// Schedules the next OBD-II request: discovers supported PIDs first, then RPM/speed
// ~1 Hz and a rotating aux slot (fuel / odometer / throttle / VIN) every 1.5 s.
// One outstanding request at a time with response timeouts.
static void pollObd() {
  const uint32_t now = millis();
  if (pendingMode) {
    if (now - requestSentAt < (isoLength ? 1000UL : 250UL) &&
        (!isoLength || now - isoAt < 250)) return;
    if (CAN_DEBUG) Serial.printf("OBD-II timeout mode=%02X pid=%02X\n", pendingMode, pendingPid);
    pendingMode = 0;
    isoLength = 0;
  }
  if (now - lastObdRequest < 100) return;
  uint8_t mode = 1;
  uint8_t pid = 0;
  if (supportPage >= 0) pid = uint8_t(supportPage);
  else if (supported[0x0C] && now - lastRpmRequest >= 1000) pid = 0x0C;
  else if (supported[0x0D] && now - lastSpeedRequest >= 1000) pid = 0x0D;
  else if (now - lastAuxRequest >= 1500) {
    lastAuxRequest = now;
    if (!vinSupportChecked) { mode = 9; pid = 0; }
    else {
      const uint8_t slot = auxSlot++ % 4;
      if (slot == 0 && supported[0x2F]) pid = 0x2F;
      else if (slot == 1 && supported[0xA6]) pid = 0xA6;
      else if (slot == 2 && supported[0x11]) pid = 0x11;
      else if (slot == 3 && vinSupported && !vehicleVin[0]) { mode = 9; pid = 2; }
      else return;
    }
  } else return;
  CanFrame request = {};
  request.identifier = obdEcu ? obdEcu - 8 : 0x7DF;
  request.data_length_code = 8;
  request.data[0] = 2;
  request.data[1] = mode;
  request.data[2] = pid;
  lastObdRequest = now;
  if (!ESP32Can.writeFrame(request)) {
    if (CAN_DEBUG) Serial.println("OBD-II CAN transmit failed");
    return;
  }
  if (pid == 0x0C && mode == 1) lastRpmRequest = now;
  if (pid == 0x0D && mode == 1) lastSpeedRequest = now;
  pendingMode = mode;
  pendingPid = pid;
  requestSentAt = now;
}

// Filters an incoming CAN frame as an OBD-II response (0x7E8-0x7EF) and dispatches
// single-frame replies, ISO-TP first frames (sends flow control), and consecutive
// frames during VIN assembly.
static void receiveObd(const CanFrame& frame) {
  if (frame.extd || frame.rtr || frame.identifier < 0x7E8 || frame.identifier > 0x7EF ||
      (obdEcu && frame.identifier != obdEcu) || frame.data_length_code < 2 ||
      frame.data_length_code > 8 || !pendingMode) return;
  const uint8_t type = frame.data[0] >> 4;
  if (type == 0) {
    const size_t length = frame.data[0] & 15;
    if (length && length + 1 <= frame.data_length_code) finishObd(frame.data + 1, length, frame.identifier);
  } else if (type == 1 && pendingMode == 9 && pendingPid == 2 && frame.data_length_code == 8) {
    const size_t length = ((frame.data[0] & 15) << 8) | frame.data[1];
    if (length != 20 || frame.data[2] != 0x49 || frame.data[3] != 2) return;
    isoLength = length;
    isoReceived = 6;
    isoSequence = 1;
    isoAt = millis();
    memcpy(isoBuffer, frame.data + 2, 6);
    CanFrame flow = {};
    flow.identifier = frame.identifier - 8;
    flow.data_length_code = 8;
    flow.data[0] = 0x30;
    flow.data[2] = 10;
    if (!ESP32Can.writeFrame(flow)) { isoLength = 0; pendingMode = 0; }
  } else if (type == 2 && isoLength) {
    if ((frame.data[0] & 15) != isoSequence || millis() - isoAt >= 250) {
      isoLength = 0;
      pendingMode = 0;
      return;
    }
    const size_t count = min(isoLength - isoReceived, size_t(frame.data_length_code - 1));
    memcpy(isoBuffer + isoReceived, frame.data + 1, count);
    isoReceived += count;
    isoSequence = (isoSequence + 1) & 15;
    isoAt = millis();
    if (isoReceived == isoLength) finishObd(isoBuffer, isoLength, frame.identifier);
  }
}
#endif

#ifdef PROTOCOL_J1939
static uint8_t vinSource = 0xFF;
static uint8_t vinSequence = 1;
static uint8_t vinBuffer[32] = {};
static size_t vinLength = 0;
static size_t vinReceived = 0;
static uint32_t vinAt = 0;

// Passively assembles a J1939 VIN from broadcast TP.CM BAM (0xEC00) announcements
// and TP.DT (0xEB00) data frames targeting PGN 0xFEEC.
static void receiveJ1939Vin(uint32_t pgn, const CanFrame& frame) {
  const uint8_t source = frame.identifier & 0xFF;
  const uint8_t destination = (frame.identifier >> 8) & 0xFF;
  if (vinLength && millis() - vinAt > 1500) vinLength = 0;
  if (pgn == 0xEC00 && destination == 0xFF && frame.data_length_code == 8 &&
      frame.data[0] == 0x20) {
    const uint32_t target = frame.data[5] | (uint32_t(frame.data[6]) << 8) | (uint32_t(frame.data[7]) << 16);
    if (target != 0xFEEC) return;
    const size_t length = frame.data[1] | (uint16_t(frame.data[2]) << 8);
    if (length < 17 || length > sizeof(vinBuffer) || frame.data[3] != (length + 6) / 7) return;
    vinSource = source;
    vinLength = length;
    vinReceived = 0;
    vinSequence = 1;
    vinAt = millis();
  } else if (pgn == 0xEB00 && destination == 0xFF && source == vinSource && vinLength &&
             frame.data_length_code == 8) {
    if (frame.data[0] != vinSequence++) { vinLength = 0; return; }
    const size_t count = min(size_t(7), vinLength - vinReceived);
    memcpy(vinBuffer + vinReceived, frame.data + 1, count);
    vinReceived += count;
    vinAt = millis();
    if (vinReceived == vinLength) { acceptVin(vinBuffer, vinLength); vinLength = 0; }
  }
}
#endif

// Brings up serial logging, BLE GATT server + advertising, the TWAI/CAN driver
// (250 kbps J1939 or 500 kbps OBD-II), and the GPS UART2 receiver.
void setup() {
  Serial.begin(115200);
  delay(1000);
  startBle();

  // Initialize CAN at 250kbps (Standard J1939 Speed)
  ESP32Can.setPins(CAN_TX, CAN_RX);
#ifdef PROTOCOL_J1939
  canReady = ESP32Can.begin(TWAI_SPEED_250KBPS);
  if (canReady) Serial.println("J1939 Receiver Ready at 250kbps!");
#else
  canReady = ESP32Can.begin(TWAI_SPEED_500KBPS);
  if (canReady) Serial.println("OBD-II polling ready: 500kbps, 11-bit, Mode 01 RPM/speed");
#endif
  if (!canReady) Serial.println("CAN Init Failed!");

  Serial2.setRxBufferSize(1024);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  // Default output includes GGA/GLL/GSA/GSV/VTG (~800+ bytes per fix) and can
  // overflow the UART buffer at the 1 s loop cadence. Keep RMC only, 1 Hz.
  sendGpsCommand("PCAS03,0,0,0,0,1,0,0,0,0,0,,,0,0");
  Serial.println("GPS UART2 ready at 9600 baud (RX2=16, TX2=17)");
}

// Runs once per LOOP_INTERVAL_S: restarts BLE advertising after disconnects,
// republishes the cached VIN, drains GPS UART2, polls OBD-II requests, then drains
// the CAN RX queue and decodes J1939 PGNs / OBD-II responses.
void loop() {
  static uint32_t lastLoopRun = 0;
  const uint32_t now = millis();
  if (now - lastLoopRun < LOOP_INTERVAL_S * 1000) return;
  lastLoopRun = now;

  if (restartAdvertising.exchange(false)) BLEDevice::startAdvertising();
  publishVin();
  pollGps();
  if (!canReady) { delay(10); return; }
  // Drain the CAN queue BEFORE pollObd(): replies arrive ~50 ms after a request
  // but sit in the queue until this 1 s tick. Draining first lets receiveObd()
  // accept them while pendingMode is still set; pollObd() then times out only
  // requests that truly got no response.
#ifdef PROTOCOL_OBD2
  while (ESP32Can.readFrame(rxFrame, CAN_READ_TIMEOUT_MS)) receiveObd(rxFrame);
#else
  while (ESP32Can.readFrame(rxFrame, CAN_READ_TIMEOUT_MS)) {
    if (!rxFrame.extd || rxFrame.rtr || rxFrame.data_length_code > 8) return;
    // 1. Extract PGN (Bits 8-25 of the 29-bit ID)
    uint32_t pgn = (rxFrame.identifier >> 8) & 0x3FFFF;
    if (((rxFrame.identifier >> 16) & 0xFF) < 240) pgn &= 0x3FF00;
    receiveJ1939Vin(pgn, rxFrame);

    Serial.printf("ID: 0x%08X | PGN: %-5u | Data: ", rxFrame.identifier, pgn);
    for (int i = 0; i < rxFrame.data_length_code; i++) {
      Serial.printf("%02X ", rxFrame.data[i]);
    }
    Serial.println();

    if (rxFrame.data_length_code < 5) return;

    // 2. Decode J1939 PGNs
    switch (pgn) {
      case 61444: {  // Engine Speed (EEC1)
        // Little Endian: Byte 3 LSB, Byte 4 MSB
        uint16_t rawRpm = (rxFrame.data[4] << 8) | rxFrame.data[3];
        float rpm = rawRpm * 0.125;
        publishMetric("rpm", rpm, lastRpmNotify);
        Serial.printf("   [DECODED] Engine Speed: %.0f RPM\n", rpm);
        break;
      }

      case 65265: {  // Vehicle Speed (CCVS1)
        // Little Endian: Byte 1 LSB, Byte 2 MSB
        uint16_t rawSpeed = (rxFrame.data[2] << 8) | rxFrame.data[1];
        float speedKmh = rawSpeed / 256.0;
        publishMetric("speed", speedKmh, lastSpeedNotify);

        // Ignition: Byte 4, Bits 1-2
        uint8_t ignitionBits = (rxFrame.data[4]) & 0x03;
        const char* ignition = (ignitionBits == 0x01) ? "ON" : "OFF";

        Serial.printf("   [DECODED] Speed: %.1f km/h | Ignition: %s\n",
                      speedKmh, ignition);
        break;
      }

      case 65217: {  // Odometer (High Res Distance)
        uint32_t rawOdo = (uint32_t)rxFrame.data[0] |
                          ((uint32_t)rxFrame.data[1] << 8) |
                          ((uint32_t)rxFrame.data[2] << 16) |
                          ((uint32_t)rxFrame.data[3] << 24);
        if (rawOdo >= 0xFA000000UL) break;
        float odoKm = rawOdo * 0.005;
        publishMetric("odo", odoKm, lastOdoNotify);
        Serial.printf("   [DECODED] Odometer: %.2f km\n", odoKm);
        break;
      }

      case 0xFEE0: {
        if (rxFrame.data_length_code != 8 || millis() - lastOdoNotify < 1000) break;
        const uint32_t raw = uint32_t(rxFrame.data[4]) | (uint32_t(rxFrame.data[5]) << 8) |
                             (uint32_t(rxFrame.data[6]) << 16) | (uint32_t(rxFrame.data[7]) << 24);
        if (raw < 0xFA000000UL) publishMetric("odo", raw * 0.125f, lastOdoNotify);
        break;
      }
      case 65266: {
        if (rxFrame.data_length_code < 7 || rxFrame.data[6] > 250) break;
        const float throttlePct = rxFrame.data[6] * 0.4f;
        publishMetric("thr", throttlePct, lastThrottleNotify);
        Serial.printf("   [DECODED] Throttle Position (SPN 51): %.1f %%\n", throttlePct);
        break;
      }

      case 61443: {  // EEC2 — Accelerator Pedal Position 1 (SPN 91, byte 2, 0.4%/bit) + Engine % Load (SPN 92, byte 3)
        if (rxFrame.data[1] <= 250) {
          float pedalPct = rxFrame.data[1] * 0.4f;
          publishMetric("pedal", pedalPct, lastPedalNotify);
          Serial.printf("   [DECODED] Accelerator Pedal: %.1f %%\n", pedalPct);
        }
        if (rxFrame.data[2] <= 0xFA) {
          float loadPct = rxFrame.data[2] - 125.0f;
          publishMetric("load", loadPct, lastLoadNotify);
          Serial.printf("   [DECODED] Engine Load: %.0f %%\n", loadPct);
        }
        break;
      }

      case 65271: {  // VEP — Key Switch Battery Potential (SPN 158), bytes 7-8, 0.05 V/bit
        uint16_t rawKey = ((uint16_t)rxFrame.data[7] << 8) | rxFrame.data[6];
        if (rawKey <= 0xFAFF) {
          float keyVolts = rawKey * 0.05f;
          publishMetric("keyv", keyVolts, lastKeyNotify);
          Serial.printf("   [DECODED] Keyswitch: %.1f V%s\n", keyVolts, keyVolts > 1.0f ? " — IGNITION ON" : "");
        }
        break;
      }

      case 0xFEFC: {  // Dash Display — Fuel Level 1 (SPN 96)
        if (rxFrame.data[1] <= 250) {
          float fuelPct = rxFrame.data[1] * 0.4f;
          publishMetric("fuel", fuelPct, lastFuelNotify);
          Serial.printf("   [DECODED] Fuel Level: %.1f %%\n", fuelPct);
        }
        break;
      }

      case 65253: {  // Engine Hours
        uint32_t rawHours = (uint32_t)rxFrame.data[0] |
                            ((uint32_t)rxFrame.data[1] << 8) |
                            ((uint32_t)rxFrame.data[2] << 16) |
                            ((uint32_t)rxFrame.data[3] << 24);
        float hours = rawHours * 0.05;
        Serial.printf("   [DECODED] Engine Hours: %.2f hrs\n", hours);
        break;
      }
    }
  }
#endif
#ifdef PROTOCOL_OBD2
  pollObd();
#endif
}
