#include <esp_now.h>
#include <WiFi.h>

constexpr uint8_t kStatusLedPin = D0;
constexpr uint8_t kAirDataEspNowDeviceId = 0x01;
constexpr uint8_t kWindEspNowDeviceId = 0x03;
constexpr uint32_t kDebugPrintIntervalMs = 500;
constexpr uint32_t kPacketStaleMs = 2000;

struct EspNowLegacyPacket {
  uint8_t  deviceId;
  uint8_t  reserved;
  uint16_t windSpeed;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};

struct EspNowAirDataPacket {
  uint8_t  deviceId;
  uint8_t  reserved;
  uint16_t windSpeed;
  uint16_t pulseCountMin;
  uint16_t pulseCountMax;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};

namespace {
EspNowAirDataPacket g_airDataPacket = {};
EspNowLegacyPacket g_windPacket = {};
unsigned long g_airDataLastReceivedAt = 0;
unsigned long g_windLastReceivedAt = 0;
unsigned long g_lastDebugAt = 0;

bool isFresh(unsigned long lastReceivedAt) {
  return (millis() - lastReceivedAt) < kPacketStaleMs;
}
}

void onEspNowReceive(const uint8_t* /*macAddr*/, const uint8_t* data, int len) {
  if (len < 1) {
    return;
  }

  const uint8_t deviceId = data[0];
  if (deviceId == kAirDataEspNowDeviceId) {
    if (len == static_cast<int>(sizeof(EspNowAirDataPacket))) {
      g_airDataPacket = *reinterpret_cast<const EspNowAirDataPacket*>(data);
    } else if (len == static_cast<int>(sizeof(EspNowLegacyPacket))) {
      const EspNowLegacyPacket* legacyPacket = reinterpret_cast<const EspNowLegacyPacket*>(data);
      g_airDataPacket.deviceId = legacyPacket->deviceId;
      g_airDataPacket.reserved = legacyPacket->reserved;
      g_airDataPacket.windSpeed = legacyPacket->windSpeed;
      g_airDataPacket.pulseCountMin = 0;
      g_airDataPacket.pulseCountMax = 0;
      g_airDataPacket.as5600Primary = legacyPacket->as5600Primary;
      g_airDataPacket.as5600Secondary = legacyPacket->as5600Secondary;
      g_airDataPacket.batteryRaw = legacyPacket->batteryRaw;
      g_airDataPacket.sequenceNumber = legacyPacket->sequenceNumber;
    } else {
      return;
    }
    g_airDataLastReceivedAt = millis();
    return;
  }

  if (deviceId == kWindEspNowDeviceId) {
    if (len != static_cast<int>(sizeof(EspNowLegacyPacket))) {
      return;
    }
    g_windPacket = *reinterpret_cast<const EspNowLegacyPacket*>(data);
    g_windLastReceivedAt = millis();
  }
}

void printStatus() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }

  g_lastDebugAt = millis();
  const bool airFresh = isFresh(g_airDataLastReceivedAt);
  const bool windFresh = isFresh(g_windLastReceivedAt);

  Serial.printf(
    "[dual_monitor] air=%s spd=%.1f pmin=%u pmax=%u as1=%u as2=%u batt=%u seq=%lu age=%lums | wind=%s spd=%.1f seq=%lu age=%lums\n",
    airFresh ? "OK" : "STALE",
    g_airDataPacket.windSpeed / 10.0f,
    g_airDataPacket.pulseCountMin,
    g_airDataPacket.pulseCountMax,
    g_airDataPacket.as5600Primary,
    g_airDataPacket.as5600Secondary,
    g_airDataPacket.batteryRaw,
    static_cast<unsigned long>(g_airDataPacket.sequenceNumber),
    static_cast<unsigned long>(millis() - g_airDataLastReceivedAt),
    windFresh ? "OK" : "STALE",
    g_windPacket.windSpeed / 10.0f,
    static_cast<unsigned long>(g_windPacket.sequenceNumber),
    static_cast<unsigned long>(millis() - g_windLastReceivedAt));
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.disconnect();

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);
    digitalWrite(kStatusLedPin, HIGH);
    Serial.println("[dual_monitor] ESP-NOW init OK");
  } else {
    Serial.println("[dual_monitor] ESP-NOW init FAILED");
  }
}

void loop() {
  printStatus();
  delay(1);
}