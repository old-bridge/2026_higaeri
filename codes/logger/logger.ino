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
uint16_t g_pendingAirspeed = 0;  // writeHreg に渡すポインタの寿命を保証
bool g_sdReady = false;
bool g_lastAirOk = false;
bool g_lastDisplayOk = false;
bool g_ledState = false;
unsigned long g_lastAirReadAt = 0;
unsigned long g_lastAirWriteAt = 0;
unsigned long g_lastDisplayReadAt = 0;
unsigned long g_lastLogAt = 0;
unsigned long g_lastLedToggleAt = 0;

// --- Modbus コールバック ---

bool cbAir(Modbus::ResultCode event, uint16_t, void*) {
  g_lastAirOk = (event == Modbus::EX_SUCCESS);
  if (!g_lastAirOk) {
    for (uint16_t& v : g_airDataBuffer) v = 0;
  }
  return true;
}

bool cbWrite(Modbus::ResultCode, uint16_t, void*) {
  return true;
}

bool cbDisplay(Modbus::ResultCode event, uint16_t, void*) {
  g_lastDisplayOk = (event == Modbus::EX_SUCCESS);
  if (!g_lastDisplayOk) {
    for (uint16_t& v : g_displayBuffer) v = 0;
  }
  return true;
}

// --- SD / ログ ---

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

void runLog() {
  if (millis() - g_lastLogAt < kLogIntervalMs) return;
  g_lastLogAt = millis();

  if (g_sdReady) {
    char record[192];
    snprintf(record,
             sizeof(record),
             "%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
             millis(),
             g_airDataBuffer[0], g_airDataBuffer[1], g_airDataBuffer[2], g_airDataBuffer[3],
             g_displayBuffer[0], g_displayBuffer[1], g_displayBuffer[2], g_displayBuffer[3], g_displayBuffer[4]);

    File file = SD.open(kLogFilePath, FILE_APPEND);
    if (file) {
      file.print(record);
      file.close();
    }
  }

  Serial.printf("[logger] sd=%s  air=%s  disp=%s  air=[spd=%u as1=%u as2=%u batt=%u]  disp=[baro=%u pot1=%u pot2=%u batt=%u ultra=%u]\n",
    g_sdReady ? "OK" : "ERR",
    g_lastAirOk ? "OK" : "ERR",
    g_lastDisplayOk ? "OK" : "ERR",
    g_airDataBuffer[0], g_airDataBuffer[1], g_airDataBuffer[2], g_airDataBuffer[3],
    g_displayBuffer[0], g_displayBuffer[1], g_displayBuffer[2], g_displayBuffer[3], g_displayBuffer[4]);
}

// --- Modbus スケジューラ ---

void runModbus() {
  if (g_master.isBusy()) return;

  const unsigned long now = millis();
  if (now - g_lastAirReadAt >= kAirReadIntervalMs) {
    g_lastAirReadAt = now;
    g_master.readHoldingRegistersAsync(kAirDataSlaveId, kAirDataReadStart, g_airDataBuffer, kAirDataReadCount, cbAir);
  } else if (now - g_lastAirWriteAt >= kAirWriteIntervalMs) {
    g_lastAirWriteAt = now;
    g_pendingAirspeed = g_airDataBuffer[0];
    g_master.writeHoldingRegistersAsync(kDisplaySlaveId, kDisplayWriteStart, &g_pendingAirspeed, 1, cbWrite);
  } else if (now - g_lastDisplayReadAt >= kDisplayReadIntervalMs) {
    g_lastDisplayReadAt = now;
    g_master.readHoldingRegistersAsync(kDisplaySlaveId, kDisplayReadStart, g_displayBuffer, kDisplayReadCount, cbDisplay);
  }
}

void updateLed() {
  if (g_lastAirOk && g_lastDisplayOk) {
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
  g_master.begin();
}

void loop() {
  g_master.task();
  runModbus();
  runLog();
  updateLed();
}
