#include <FS.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>

#include "ModbusConfig.h"
#include "ModbusMaster.h"

constexpr uint8_t kRs485DePin = D2;
constexpr uint8_t kModbusRxPin = D7;  // GPIO20 (UART0 default RX)
constexpr uint8_t kModbusTxPin = D6;  // GPIO21 (UART0 default TX)
constexpr uint8_t kLoggerLedPin = D1;
constexpr uint8_t kSpiSckPin = 8;
constexpr uint8_t kSpiMisoPin = 9;
constexpr uint8_t kSpiMosiPin = 10;
constexpr uint8_t kSdChipSelectPin = 5;

namespace {
constexpr const char* kLogFilePath = "/flight_log.csv";
constexpr unsigned long kLedBlinkIntervalMs = 500;

HardwareSerial MySerial0(0);
ModbusMaster g_master(MySerial0, kRs485DePin, kModbusRxPin, kModbusTxPin);
uint16_t g_airDataBuffer[kAirDataReadCount] = {0};
uint16_t g_displayBuffer[kDisplayReadCount] = {0};
bool g_sdReady = false;
bool g_lastCycleOk = false;
bool g_ledState = false;
unsigned long g_lastPollAt = 0;
unsigned long g_lastLedToggleAt = 0;

void clearBuffers() {
  for (uint16_t& value : g_airDataBuffer) {
    value = 0;
  }
  for (uint16_t& value : g_displayBuffer) {
    value = 0;
  }
}

void initSdCard() {
  if (!SD.begin(kSdChipSelectPin)) {
    g_sdReady = false;
    return;
  }

  if (!SD.exists(kLogFilePath)) {
    File file = SD.open(kLogFilePath, FILE_WRITE);
    if (file) {
      file.println("timestamp,airspeed,as5600_1,as5600_2,air_battery,baro_alt,pot1,pot2,display_battery,ultrasonic");
      file.close();
    }
  }

  g_sdReady = true;
}

void pollDevices() {
  g_lastCycleOk = true;

  if (!g_master.readHoldingRegisters(kAirDataSlaveId, kAirDataReadStart, g_airDataBuffer, kAirDataReadCount)) {
    g_lastCycleOk = false;
    for (uint16_t& value : g_airDataBuffer) {
      value = 0;
    }
  }

  const uint16_t airspeed = g_airDataBuffer[0];
  if (!g_master.writeHoldingRegisters(kDisplaySlaveId, kDisplayWriteStart, &airspeed, 1)) {
    g_lastCycleOk = false;
  }

  if (!g_master.readHoldingRegisters(kDisplaySlaveId, kDisplayReadStart, g_displayBuffer, kDisplayReadCount)) {
    g_lastCycleOk = false;
    for (uint16_t& value : g_displayBuffer) {
      value = 0;
    }
  }
}

void writeLogRecord() {
  if (!g_sdReady) {
    return;
  }

  char record[192];
  snprintf(record,
           sizeof(record),
           "%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
           millis(),
           g_airDataBuffer[0],
           g_airDataBuffer[1],
           g_airDataBuffer[2],
           g_airDataBuffer[3],
           g_displayBuffer[0],
           g_displayBuffer[1],
           g_displayBuffer[2],
           g_displayBuffer[3],
           g_displayBuffer[4]);

  File file = SD.open(kLogFilePath, FILE_APPEND);
  if (!file) {
    g_lastCycleOk = false;
    return;
  }

  if (!file.print(record)) {
    g_lastCycleOk = false;
  }
  file.close();
}

void updateLed() {
  if (g_lastCycleOk) {
    if (millis() - g_lastLedToggleAt >= kLedBlinkIntervalMs) {
      g_lastLedToggleAt = millis();
      g_ledState = !g_ledState;
      digitalWrite(kLoggerLedPin, g_ledState ? HIGH : LOW);
    }
    return;
  }

  digitalWrite(kLoggerLedPin, HIGH);
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(100);

  pinMode(kLoggerLedPin, OUTPUT);
  digitalWrite(kLoggerLedPin, LOW);

  SPI.begin(kSpiSckPin, kSpiMisoPin, kSpiMosiPin, kSdChipSelectPin);
  initSdCard();
  clearBuffers();
  g_master.begin();
}

void loop() {
  g_master.task();

  if (millis() - g_lastPollAt >= kLoggerPollIntervalMs) {
    g_lastPollAt = millis();
    pollDevices();
    writeLogRecord();
    Serial.printf("[logger] sd=%s  cycle=%s  air=[spd=%u as1=%u as2=%u batt=%u]  disp=[baro=%u pot1=%u pot2=%u batt=%u ultra=%u]\n",
      g_sdReady ? "OK" : "ERR",
      g_lastCycleOk ? "OK" : "ERR",
      g_airDataBuffer[0], g_airDataBuffer[1], g_airDataBuffer[2], g_airDataBuffer[3],
      g_displayBuffer[0], g_displayBuffer[1], g_displayBuffer[2], g_displayBuffer[3], g_displayBuffer[4]);
  }

  updateLed();
}
