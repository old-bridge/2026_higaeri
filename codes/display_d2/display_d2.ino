#include <Adafruit_DPS310.h>
#include <Wire.h>

#include "DisplayD2Config.h"

constexpr uint8_t kPot1Pin = A0;
constexpr uint8_t kPot2Pin = A1;
constexpr uint8_t kBatteryPin = A2;
constexpr uint8_t kUltrasonicRxPin = D0;
constexpr uint8_t kUltrasonicTxPin = D1;
constexpr uint8_t kStatusLedPin = D10;

struct DisplayD2Payload {
  uint16_t potentiometer1;
  uint16_t potentiometer2;
  uint16_t batteryVoltage;
  uint16_t ultrasonicAlt;
  uint16_t baroAlt;
};

namespace {
Adafruit_DPS310 g_dps;
bool g_dpsReady = false;
DisplayD2Payload g_payload = {0, 0, 0, 0, 0};
unsigned long g_lastUpdateAt = 0;

void writePayload() {
  uint8_t buffer[kDisplayD2PayloadSize] = {0};
  buffer[0] = static_cast<uint8_t>(g_payload.potentiometer1 & 0xFF);
  buffer[1] = static_cast<uint8_t>(g_payload.potentiometer1 >> 8);
  buffer[2] = static_cast<uint8_t>(g_payload.potentiometer2 & 0xFF);
  buffer[3] = static_cast<uint8_t>(g_payload.potentiometer2 >> 8);
  buffer[4] = static_cast<uint8_t>(g_payload.batteryVoltage & 0xFF);
  buffer[5] = static_cast<uint8_t>(g_payload.batteryVoltage >> 8);
  buffer[6] = static_cast<uint8_t>(g_payload.ultrasonicAlt & 0xFF);
  buffer[7] = static_cast<uint8_t>(g_payload.ultrasonicAlt >> 8);
  buffer[8] = static_cast<uint8_t>(g_payload.baroAlt & 0xFF);
  buffer[9] = static_cast<uint8_t>(g_payload.baroAlt >> 8);
  Wire.write(buffer, kDisplayD2PayloadSize);
}

void updateBarometricAltitude() {
  if (!g_dpsReady) {
    g_payload.baroAlt = 0;
    return;
  }

  sensors_event_t temperatureEvent;
  sensors_event_t pressureEvent;
  if (!g_dps.getEvents(&temperatureEvent, &pressureEvent)) {
    g_payload.baroAlt = 0;
    return;
  }

  const float altitudeMeters = g_dps.readAltitude(kSeaLevelPressureHpa);
  g_payload.baroAlt = static_cast<uint16_t>(altitudeMeters * 10.0f);
}

void updateSensors() {
  if (millis() - g_lastUpdateAt < kDisplayD2SensorIntervalMs) {
    return;
  }
  g_lastUpdateAt = millis();

  g_payload.potentiometer1 = analogRead(kPot1Pin);
  g_payload.potentiometer2 = analogRead(kPot2Pin);
  g_payload.batteryVoltage = analogRead(kBatteryPin);
  g_payload.ultrasonicAlt = 0;
  updateBarometricAltitude();
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, HIGH);
  delay(100);

  pinMode(kPot1Pin, INPUT);
  pinMode(kPot2Pin, INPUT);
  pinMode(kBatteryPin, INPUT);
  pinMode(kUltrasonicRxPin, INPUT);
  pinMode(kUltrasonicTxPin, OUTPUT);
  digitalWrite(kUltrasonicTxPin, LOW);

  Wire.begin(kDisplayD2I2CAddress);
  Wire.onRequest(writePayload);

  g_dpsReady = g_dps.begin_I2C(kDps310Address, &Wire);
  if (g_dpsReady) {
    g_dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    g_dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }
}

void loop() {
  updateSensors();
}
