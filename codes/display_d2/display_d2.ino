#include <Wire.h>

#include "DisplayD2Config.h"

constexpr uint8_t kPot1Pin = A0;
constexpr uint8_t kPot2Pin = A1;
constexpr uint8_t kBatteryPin = A2;
constexpr uint8_t kUltrasonicRxPin = D0;
constexpr uint8_t kUltrasonicTxPin = D1;
constexpr uint8_t kStatusLedPin = D10;
constexpr uint8_t kBuzzerPin = D3;

struct DisplayD2Payload {
  uint16_t potentiometer1;
  uint16_t potentiometer2;
  uint16_t batteryVoltage;
  uint16_t ultrasonicAlt;
};


constexpr uint32_t kDebugPrintIntervalMs = 1000;

namespace {
DisplayD2Payload g_payload = {0, 0, 0, 0};
unsigned long g_lastUpdateAt = 0;
unsigned long g_lastDebugAt = 0;
volatile bool g_rollAlarm = false;

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
  Wire.write(buffer, kDisplayD2PayloadSize);
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
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }
  g_lastDebugAt = millis();
  Serial.printf("[display_d2] pot1=%u  pot2=%u  batt=%u  ultra=%u  roll_alarm=%s\n",
    g_payload.potentiometer1,
    g_payload.potentiometer2,
    g_payload.batteryVoltage,
    g_payload.ultrasonicAlt,
    g_rollAlarm ? "ON" : "OFF");
}
}

void onI2CReceive(int len) {
  if (len < 1) return;
  g_rollAlarm = (Wire.read() != 0);
  while (Wire.available()) Wire.read();  // 余分バイトを捨てる
}

void setup() {
  Serial.begin(kDebugBaudRate);
  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, HIGH);
  delay(100);

  pinMode(kBuzzerPin, OUTPUT);
  pinMode(kPot1Pin, INPUT);
  pinMode(kPot2Pin, INPUT);
  pinMode(kBatteryPin, INPUT);
  pinMode(kUltrasonicRxPin, INPUT);
  pinMode(kUltrasonicTxPin, OUTPUT);
  digitalWrite(kUltrasonicTxPin, LOW);

  Wire.begin(kDisplayD2I2CAddress);
  Wire.onRequest(writePayload);
  Wire.onReceive(onI2CReceive);
}

void loop() {
  updateSensors();
  printDebug();

  if (g_rollAlarm) {
    tone(kBuzzerPin, 1000);
  } else {
    noTone(kBuzzerPin);
  }
}
