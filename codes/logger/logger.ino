#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <FS.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>

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

// RS485 が連続でこの回数失敗したら ESP-NOW にフォールバック
constexpr uint32_t kModbusFailureThreshold = 3;
// ESP-NOW パケットがこの時間 (ms) 以内に届いていれば有効とみなす
constexpr uint32_t kEspNowStaleMs = 2000;
constexpr uint8_t kAirDataEspNowDeviceId = 0x01;
constexpr uint8_t kWindEspNowDeviceId = 0x03;

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
bool g_lastCycleOk = true;
bool g_ledState = false;
unsigned long g_lastAirReadAt = 0;
unsigned long g_lastAirWriteAt = 0;
unsigned long g_lastDisplayReadAt = 0;
unsigned long g_lastLogAt = 0;
unsigned long g_lastPollAt = 0;
unsigned long g_lastLedToggleAt = 0;
EspNowAirDataPacket g_airEspNowLatest = {};
EspNowLegacyPacket g_windEspNowLatest = {};
unsigned long g_airEspNowLastReceivedAt = 0;
unsigned long g_windEspNowLastReceivedAt = 0;
uint32_t g_modbusConsecutiveFailures = 0;
unsigned long g_lastAlarmWriteAt = 0;
uint16_t g_lastRollAlarmValue = 0xFFFF;

// ---- DS3231 RTC (Wire直接使用) ----------------------------------------
constexpr uint8_t kRtcI2cAddr = 0x68;
bool g_rtcReady = false;

static uint8_t bcdEncode(uint8_t val) { return ((val / 10) << 4) | (val % 10); }
static uint8_t bcdDecode(uint8_t bcd) { return (bcd >> 4) * 10 + (bcd & 0x0F); }

// OSF フラグを確認し、オシレータが止まっていた場合のみコンパイル時刻を書き込む
static void initRtc() {
  // OSF フラグ（ステータスレジスタ 0x0F の bit7）を読む
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x0F);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)kRtcI2cAddr, (uint8_t)1) < 1) {
    // I2C 通信自体が失敗（RTC 未接続など）
    g_rtcReady = false;
    return;
  }
  const uint8_t statusReg = Wire.read();
  g_rtcReady = true;

  if ((statusReg & 0x80) == 0) {
    // OSF=0: オシレータは止まっていない → 時刻は有効なのでそのまま使う
    Serial.println("[logger] RTC time is valid, skipping write");
    return;
  }

  // OSF=1: 電源断などで時刻が無効 → コンパイル時刻を書き込む
  Serial.println("[logger] RTC oscillator stopped, writing compile time");
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {__DATE__[0], __DATE__[1], __DATE__[2], '\0'};
  const char* found = strstr(months, mon);
  uint8_t month = found ? static_cast<uint8_t>((found - months) / 3 + 1) : 1;
  uint8_t day   = (__DATE__[4] == ' ' ? 0 : __DATE__[4] - '0') * 10 + (__DATE__[5] - '0');
  uint8_t year  = (__DATE__[9] - '0') * 10 + (__DATE__[10] - '0');
  uint8_t hour  = (__TIME__[0] - '0') * 10 + (__TIME__[1] - '0');
  uint8_t min_  = (__TIME__[3] - '0') * 10 + (__TIME__[4] - '0');
  uint8_t sec   = (__TIME__[6] - '0') * 10 + (__TIME__[7] - '0');

  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x00);
  Wire.write(bcdEncode(sec));
  Wire.write(bcdEncode(min_));
  Wire.write(bcdEncode(hour));
  Wire.write(0x01);              // 曜日（未使用）
  Wire.write(bcdEncode(day));
  Wire.write(bcdEncode(month));
  Wire.write(bcdEncode(year));
  if (Wire.endTransmission() != 0) {
    g_rtcReady = false;
    return;
  }

  // OSF フラグをクリア
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x0F);
  Wire.write(statusReg & ~0x80);
  Wire.endTransmission();
}

// "YYYY-MM-DD HH:MM:SS" を取得。失敗時は "BOOT+XXXXXms" にフォールバック
static void getRtcTimestamp(char* buf, size_t bufLen) {
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)kRtcI2cAddr, (uint8_t)7) < 7) {
    snprintf(buf, bufLen, "BOOT+%lums", millis());
    return;
  }
  uint8_t sec   = bcdDecode(Wire.read() & 0x7F);
  uint8_t min_  = bcdDecode(Wire.read() & 0x7F);
  uint8_t hour  = bcdDecode(Wire.read() & 0x3F);
  Wire.read();  // 曜日（スキップ）
  uint8_t date  = bcdDecode(Wire.read() & 0x3F);
  uint8_t month = bcdDecode(Wire.read() & 0x1F);
  uint8_t year  = bcdDecode(Wire.read());
  snprintf(buf, bufLen, "20%02u-%02u-%02u %02u:%02u:%02u",
           year, month, date, hour, min_, sec);
}

// ---- BNO085 IMU (I2C) -------------------------------------------------
Adafruit_BNO08x g_bno08x(-1);
bool g_imuReady = false;

struct {
  float roll  = 0.0f;
  float pitch = 0.0f;
  float yaw   = 0.0f;
} g_imuData;

static void readIMUData() {
  if (!g_imuReady) return;

  if (g_bno08x.wasReset()) {
    g_imuReady = g_bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
    return;
  }

  sh2_SensorValue_t ev;
  if (!g_bno08x.getSensorEvent(&ev)) return;
  if (ev.sensorId != SH2_ARVR_STABILIZED_RV) return;

  const float qr = ev.un.arvrStabilizedRV.real;
  const float qi = ev.un.arvrStabilizedRV.i;
  const float qj = ev.un.arvrStabilizedRV.j;
  const float qk = ev.un.arvrStabilizedRV.k;
  const float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;

  g_imuData.yaw   = atan2(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
  g_imuData.pitch = asin(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  g_imuData.roll  = atan2(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
}

static void writeRollAlarm() {
  const uint16_t alarm = (g_imuData.roll > 5.0f || g_imuData.roll < -5.0f) ? uint16_t(1) : uint16_t(0);
  if (alarm == g_lastRollAlarmValue) {
    return;
  }

  if (g_master.writeHoldingRegisters(kDisplaySlaveId, kDisplayRollAlarmWriteRegister, &alarm, 1)) {
    g_lastRollAlarmValue = alarm;
  } else {
    g_lastCycleOk = false;
  }
}

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
      file.println("timestamp,airspeed,pulse_min,pulse_max,wind_board_airspeed,as5600_1,as5600_2,air_battery,baro_alt,pot1,pot2,display_battery,ultrasonic,roll,pitch,yaw");
      file.close();
    }
  }

  g_sdReady = true;
}

void pollDevices() {
  g_lastCycleOk = true;
  g_lastAirOk = g_master.readHoldingRegisters(kAirDataSlaveId, kAirDataReadStart, g_airDataBuffer, kAirDataReadCount);
  if (!g_lastAirOk) {
    g_modbusConsecutiveFailures++;
    g_lastCycleOk = false;
    const bool espNowFresh = (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
    if (g_modbusConsecutiveFailures >= kModbusFailureThreshold && espNowFresh) {
      // RS485 失敗: ESP-NOW の最終受信値をフォールバックとして使用
      g_airDataBuffer[0] = g_airEspNowLatest.windSpeed;
      g_airDataBuffer[1] = g_airEspNowLatest.as5600Primary;
      g_airDataBuffer[2] = g_airEspNowLatest.as5600Secondary;
      g_airDataBuffer[3] = g_airEspNowLatest.batteryRaw;
    } else {
      for (uint16_t& value : g_airDataBuffer) {
        value = 0;
      }
    }
  } else {
    g_modbusConsecutiveFailures = 0;
  }

  const uint16_t airspeed = g_airDataBuffer[0];
  g_lastDisplayOk = g_master.writeHoldingRegisters(kDisplaySlaveId, kDisplayAirspeedWriteRegister, &airspeed, 1);
  if (!g_lastDisplayOk) {
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
  if (millis() - g_lastLogAt < kLogIntervalMs) return;
  g_lastLogAt = millis();

  char timestamp[24];
  getRtcTimestamp(timestamp, sizeof(timestamp));

  const bool airEspNowFresh = (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
  const uint16_t airPulseMin = airEspNowFresh ? g_airEspNowLatest.pulseCountMin : 0;
  const uint16_t airPulseMax = airEspNowFresh ? g_airEspNowLatest.pulseCountMax : 0;

  char record[320];
  snprintf(record,
           sizeof(record),
           "%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f\n",
           timestamp,
           g_airDataBuffer[0],
           airPulseMin,
           airPulseMax,
           g_windEspNowLatest.windSpeed,
           g_airDataBuffer[1],
           g_airDataBuffer[2],
           g_airDataBuffer[3],
           g_displayBuffer[0],
           g_displayBuffer[1],
           g_displayBuffer[2],
           g_displayBuffer[3],
           g_displayBuffer[4],
           g_imuData.roll, g_imuData.pitch, g_imuData.yaw);

  if (!g_sdReady) {
    g_lastCycleOk = false;
    return;
  }

  File file = SD.open(kLogFilePath, FILE_APPEND);
  if (!file) {
    g_lastCycleOk = false;
    g_sdReady = false;
    return;
  }

  file.print(record);
  file.close();
}

}  // namespace

void onEspNowReceive(const uint8_t* macAddr, const uint8_t* data, int len) {
  if (len < 1) {
    return;
  }
  const uint8_t deviceId = data[0];
  if (deviceId == kAirDataEspNowDeviceId) {
    if (len == static_cast<int>(sizeof(EspNowAirDataPacket))) {
      g_airEspNowLatest = *reinterpret_cast<const EspNowAirDataPacket*>(data);
    } else if (len == static_cast<int>(sizeof(EspNowLegacyPacket))) {
      const EspNowLegacyPacket* legacyPacket = reinterpret_cast<const EspNowLegacyPacket*>(data);
      g_airEspNowLatest.deviceId = legacyPacket->deviceId;
      g_airEspNowLatest.reserved = legacyPacket->reserved;
      g_airEspNowLatest.windSpeed = legacyPacket->windSpeed;
      g_airEspNowLatest.pulseCountMin = 0;
      g_airEspNowLatest.pulseCountMax = 0;
      g_airEspNowLatest.as5600Primary = legacyPacket->as5600Primary;
      g_airEspNowLatest.as5600Secondary = legacyPacket->as5600Secondary;
      g_airEspNowLatest.batteryRaw = legacyPacket->batteryRaw;
      g_airEspNowLatest.sequenceNumber = legacyPacket->sequenceNumber;
    } else {
      return;
    }
    g_airEspNowLastReceivedAt = millis();
    return;
  }
  if (deviceId == kWindEspNowDeviceId) {
    if (len != static_cast<int>(sizeof(EspNowLegacyPacket))) {
      return;
    }
    g_windEspNowLatest = *reinterpret_cast<const EspNowLegacyPacket*>(data);
    g_windEspNowLastReceivedAt = millis();
  }
}

namespace {
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

  Wire.begin();
  initRtc();
  Serial.printf("[logger] RTC %s\n", g_rtcReady ? "OK" : "FAILED");

  if (g_bno08x.begin_I2C()) {
    g_imuReady = g_bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
    Serial.printf("[logger] BNO085 %s\n", g_imuReady ? "OK" : "report FAILED");
  } else {
    Serial.println("[logger] BNO085 init FAILED");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[logger] ESP-NOW init OK");
  } else {
    Serial.println("[logger] ESP-NOW init FAILED");
  }
}

void loop() {
  g_master.task();
  readIMUData();

  if (millis() - g_lastAlarmWriteAt >= kRollAlarmWriteIntervalMs) {
    g_lastAlarmWriteAt = millis();
    writeRollAlarm();
  }

  if (millis() - g_lastPollAt >= kLoggerPollIntervalMs) {
    g_lastPollAt = millis();
    pollDevices();
    writeLogRecord();
    const bool usingFallback = g_modbusConsecutiveFailures >= kModbusFailureThreshold
                              && (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
    char ts[24];
    getRtcTimestamp(ts, sizeof(ts));
    Serial.printf("[logger] %s  sd=%s  cycle=%s  src=%s  air=[spd=%u pmin=%u pmax=%u as1=%u as2=%u batt=%u]  wind=[spd=%u seq=%lu]  disp=[baro=%u pot1=%u pot2=%u batt=%u ultra=%u]  imu=[r=%.1f p=%.1f y=%.1f]\n",
      ts,
      g_sdReady ? "OK" : "ERR",
      g_lastCycleOk ? "OK" : "ERR",
      usingFallback ? "ESPNOW" : "RS485",
      g_airDataBuffer[0], g_airEspNowLatest.pulseCountMin, g_airEspNowLatest.pulseCountMax,
      g_airDataBuffer[1], g_airDataBuffer[2], g_airDataBuffer[3],
      g_windEspNowLatest.windSpeed, static_cast<unsigned long>(g_windEspNowLatest.sequenceNumber),
      g_displayBuffer[0], g_displayBuffer[1], g_displayBuffer[2], g_displayBuffer[3], g_displayBuffer[4],
      g_imuData.roll, g_imuData.pitch, g_imuData.yaw);
  }

  updateLed();
}
