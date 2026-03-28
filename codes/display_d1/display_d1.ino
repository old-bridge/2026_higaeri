#include <HardwareSerial.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include "ModbusConfig.h"
#include "ModbusSlave.h"

constexpr uint8_t kRs485DePin = D3;
constexpr uint8_t kModbusRxPin = 5;
constexpr uint8_t kModbusTxPin = 4;
constexpr uint8_t kSpiSckPin = 8;
constexpr uint8_t kSpiMisoPin = 9;
constexpr uint8_t kSpiMosiPin = 10;
constexpr uint16_t kDisplayBackground = 0x5AEB;

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
DisplayD2Snapshot g_snapshot = {0, 0, 0, 0, 0, false, 0, 0};
unsigned long g_lastPollAt = 0;
unsigned long g_lastRenderAt = 0;

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
  g_snapshot.baroAlt = static_cast<uint16_t>((buffer[9] << 8) | buffer[8]);
  g_snapshot.connected = true;
  g_snapshot.successCount++;
}

void renderDisplay() {
  if (millis() - g_lastRenderAt < kDisplayRefreshIntervalMs) {
    return;
  }
  g_lastRenderAt = millis();

  g_tft.fillScreen(kDisplayBackground);
  g_tft.setTextColor(TFT_WHITE, kDisplayBackground);
  g_tft.drawCentreString("DISPLAY D1", 120, 8, 2);

  char line[48];
  int y = 30;

  snprintf(line, sizeof(line), "Airspeed : %u", g_slave.airspeed());
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  snprintf(line, sizeof(line), "Baro Alt : %u", g_snapshot.baroAlt);
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  snprintf(line, sizeof(line), "Pot1/Pot2: %u / %u", g_snapshot.potentiometer1, g_snapshot.potentiometer2);
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  snprintf(line, sizeof(line), "Battery  : %u", g_snapshot.batteryVoltage);
  g_tft.drawString(line, 8, y, 2);
  y += 20;

  snprintf(line, sizeof(line), "Ultrason.: %u", g_snapshot.ultrasonicAlt);
  g_tft.drawString(line, 8, y, 2);
  y += 26;

  snprintf(line, sizeof(line), "I2C %s", g_snapshot.connected ? "OK" : "ERROR");
  g_tft.setTextColor(g_snapshot.connected ? TFT_GREEN : TFT_RED, kDisplayBackground);
  g_tft.drawString(line, 8, y, 2);
  g_tft.setTextColor(TFT_WHITE, kDisplayBackground);
  y += 20;

  snprintf(line,
           sizeof(line),
           "OK:%lu ERR:%lu",
           static_cast<unsigned long>(g_snapshot.successCount),
           static_cast<unsigned long>(g_snapshot.failureCount));
  g_tft.drawString(line, 8, y, 2);
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(100);

  Wire.begin();
  Wire.setClock(400000);

  SPI.begin(kSpiSckPin, kSpiMisoPin, kSpiMosiPin);
  g_tft.init();
  g_tft.setRotation(0);
  g_tft.fillScreen(kDisplayBackground);

  g_slave.begin();
}

void loop() {
  pollDisplayD2();
  g_slave.setSensors(g_snapshot);
  g_slave.task();
  renderDisplay();
}
