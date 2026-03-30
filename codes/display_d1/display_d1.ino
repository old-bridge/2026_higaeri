#include <Adafruit_DPS310.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include "ModbusConfig.h"
#include "ModbusSlave.h"

constexpr uint8_t kRs485DePin = D3;
constexpr uint8_t kModbusRxPin = D7;  // GPIO20 (UART0 default RX)
constexpr uint8_t kModbusTxPin = D6;  // GPIO21 (UART0 default TX)
constexpr uint8_t kSpiSckPin = 8;
constexpr uint8_t kSpiMisoPin = 9;
constexpr uint8_t kSpiMosiPin = 10;
constexpr uint16_t kDisplayBackground = 0x5AEB;
constexpr uint32_t kDebugPrintIntervalMs = 1000;
constexpr uint8_t kDps310I2CAddress = 0x76;
constexpr float kSeaLevelPressureHpa = 1013.25f;

struct DisplayD2Snapshot {
  uint16_t potentiometer1;
  uint16_t potentiometer2;
  uint16_t batteryVoltage;
  uint16_t ultrasonicAlt;
  uint16_t baroAlt;
  bool connected;
  uint32_t successCount;
  uint32_t failureCount;
};

class DisplayD1SlaveNode : public ModbusSlaveBase {
public:
  explicit DisplayD1SlaveNode(HardwareSerial& serial)
    : ModbusSlaveBase(serial, kDisplaySlaveId, kRs485DePin, kModbusRxPin, kModbusTxPin) {}

  void setSensors(const DisplayD2Snapshot& snapshot) {
    sensors_ = snapshot;
  }

  uint16_t airspeed() const {
    return airspeed_;
  }

  void task() {
    setHoldingValue(kDisplayBaroRegister, sensors_.baroAlt);
    setHoldingValue(kDisplayPot1Register, sensors_.potentiometer1);
    setHoldingValue(kDisplayPot2Register, sensors_.potentiometer2);
    setHoldingValue(kDisplayBatteryRegister, sensors_.batteryVoltage);
    setHoldingValue(kDisplayUltrasonicRegister, sensors_.ultrasonicAlt);
    ModbusSlaveBase::task();
  }

protected:
  void setupRegisters() override {
    addHoldingRegisters(kDisplayReadStart, kDisplayReadCount, 0);
    addHoldingRegisters(kDisplayWriteStart, kDisplayWriteCount, 0);
  }

  void setupCallbacks() override {
    registerReadHandler(kDisplayReadStart, kDisplayReadCount);
    registerWriteHandler(kDisplayWriteStart, kDisplayWriteCount);
  }

  uint16_t onWriteRegister(TRegister* reg, uint16_t newValue) override {
    if (reg->address.address == kDisplayWriteStart) {
      airspeed_ = newValue;
    }
    return newValue;
  }

private:
  DisplayD2Snapshot sensors_ = {0, 0, 0, 0, 0, false, 0, 0};
  uint16_t airspeed_ = 0;
};

namespace {
HardwareSerial MySerial0(0);
DisplayD1SlaveNode g_slave(MySerial0);
TFT_eSPI g_tft;
Adafruit_DPS310 g_dps;
bool g_dpsReady = false;
DisplayD2Snapshot g_snapshot = {0, 0, 0, 0, 0, false, 0, 0};
unsigned long g_lastPollAt = 0;
unsigned long g_lastRenderAt = 0;
unsigned long g_lastDebugAt = 0;

void pollDisplayD2() {
  if (millis() - g_lastPollAt < kDisplayI2CPollIntervalMs) {
    return;
  }
  g_lastPollAt = millis();

  Wire.beginTransmission(kDisplayD2I2CAddress);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    g_snapshot = {0, 0, 0, 0, 0, false, g_snapshot.successCount, g_snapshot.failureCount + 1};
    return;
  }

  const int receivedBytes = Wire.requestFrom(static_cast<int>(kDisplayD2I2CAddress), static_cast<int>(kDisplayD2PayloadSize));
  if (receivedBytes != kDisplayD2PayloadSize) {
    g_snapshot = {0, 0, 0, 0, 0, false, g_snapshot.successCount, g_snapshot.failureCount + 1};
    return;
  }

  uint8_t buffer[kDisplayD2PayloadSize];
  for (uint8_t index = 0; index < kDisplayD2PayloadSize; ++index) {
    buffer[index] = Wire.read();
  }

  g_snapshot.potentiometer1 = static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
  g_snapshot.potentiometer2 = static_cast<uint16_t>((buffer[3] << 8) | buffer[2]);
  g_snapshot.batteryVoltage = static_cast<uint16_t>((buffer[5] << 8) | buffer[4]);
  g_snapshot.ultrasonicAlt = static_cast<uint16_t>((buffer[7] << 8) | buffer[6]);
  g_snapshot.connected = true;
  g_snapshot.successCount++;
}

void updateBarometricAltitude() {
  if (!g_dpsReady) {
    g_snapshot.baroAlt = 0;
    return;
  }

  sensors_event_t temperatureEvent;
  sensors_event_t pressureEvent;
  if (!g_dps.getEvents(&temperatureEvent, &pressureEvent)) {
    g_snapshot.baroAlt = 0;
    return;
  }

  const float altitudeMeters = g_dps.readAltitude(kSeaLevelPressureHpa);
  g_snapshot.baroAlt = static_cast<uint16_t>(altitudeMeters * 10.0f);
}

void renderDisplay() {
  if (millis() - g_lastRenderAt < kDisplayRefreshIntervalMs) {
    return;
  }
  g_lastRenderAt = millis();

  // fillScreen() は 240x320 全ピクセルの SPI 書き込みで 20-50ms ブロックする。
  // その間 task() が呼ばれず Modbus フレームを取りこぼす。
  // 代わりに setTextColor(fg, bg) の背景色上書きで古い文字を消す。
  g_tft.setTextColor(TFT_WHITE, kDisplayBackground);

  // 固定幅文字列で上書きすることで fillScreen 不要
  char line[48];
  int y = 8;

  g_tft.drawCentreString("DISPLAY D1", 120, y, 2);
  y = 30;

  g_slave.task();  // Modbus フレームを逃さない

  snprintf(line, sizeof(line), "Airspeed : %-6u", g_slave.airspeed());
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  snprintf(line, sizeof(line), "Baro Alt : %-6u", g_snapshot.baroAlt);
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  g_slave.task();

  snprintf(line, sizeof(line), "Pot1/Pot2: %-5u / %-5u", g_snapshot.potentiometer1, g_snapshot.potentiometer2);
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  snprintf(line, sizeof(line), "Battery  : %-6u", g_snapshot.batteryVoltage);
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  g_slave.task();

  snprintf(line, sizeof(line), "Ultrason.: %-6u", g_snapshot.ultrasonicAlt);
  g_tft.drawString(line, 8, y, 2);
  y += 26;

  snprintf(line, sizeof(line), "I2C %-8s", g_snapshot.connected ? "OK" : "ERROR");
  g_tft.setTextColor(g_snapshot.connected ? TFT_GREEN : TFT_RED, kDisplayBackground);
  g_tft.drawString(line, 8, y, 2);
  g_tft.setTextColor(TFT_WHITE, kDisplayBackground);
  y += 20;

  snprintf(line,
           sizeof(line),
           "OK:%-6lu ERR:%-6lu",
           static_cast<unsigned long>(g_snapshot.successCount),
           static_cast<unsigned long>(g_snapshot.failureCount));
  g_tft.drawString(line, 8, y, 2);
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }
  g_lastDebugAt = millis();
  Serial.printf("[display_d1] airspeed=%u  baro=%u  pot1=%u  pot2=%u  batt=%u  ultra=%u  i2c=%s  ok=%lu  err=%lu\n",
    g_slave.airspeed(),
    g_snapshot.baroAlt,
    g_snapshot.potentiometer1,
    g_snapshot.potentiometer2,
    g_snapshot.batteryVoltage,
    g_snapshot.ultrasonicAlt,
    g_snapshot.connected ? "OK" : "ERR",
    static_cast<unsigned long>(g_snapshot.successCount),
    static_cast<unsigned long>(g_snapshot.failureCount));
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(100);

  Wire.begin();
  Wire.setClock(400000);

  g_dpsReady = g_dps.begin_I2C(kDps310I2CAddress, &Wire);
  if (g_dpsReady) {
    g_dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    g_dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }

  SPI.begin(kSpiSckPin, kSpiMisoPin, kSpiMosiPin);
  g_tft.init();
  g_tft.setRotation(0);
  g_tft.fillScreen(kDisplayBackground);

  g_slave.begin();
}

void loop() {
  pollDisplayD2();
  updateBarometricAltitude();
  g_slave.setSensors(g_snapshot);
  g_slave.task();
  renderDisplay();
  printDebug();
}
