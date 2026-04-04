#include <Adafruit_DPS310.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include "ModbusConfig.h"
#include "ModbusSlave.h"
#include "font_data.h"

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

// ── Gauge Layout ──
constexpr int16_t kGaugeCx = 120;
constexpr int16_t kGaugeCy = 125;
constexpr int16_t kGaugeOuterR = 105;
constexpr int16_t kArcOuterR = 103;
constexpr int16_t kArcInnerR = 88;
constexpr int16_t kTickOuterR = 105;
constexpr int16_t kTickMajorInnerR = 85;
constexpr int16_t kTickMinorInnerR = 95;
constexpr int16_t kNeedleLen = 75;
constexpr int16_t kNumberR = 72;
constexpr float kGaugeStartDeg = 30.0f;
constexpr float kGaugeSweepDeg = 300.0f;
constexpr float kGaugeMaxSpeed = 10.0f;

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

  uint16_t rollAlarm() const {
    return rollAlarm_;
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
    if (reg->address.address == kDisplayAirspeedWriteRegister) {
      airspeed_ = newValue;
    } else if (reg->address.address == kDisplayRollAlarmWriteRegister) {
      rollAlarm_ = newValue;
    }
    return newValue;
  }

private:
  DisplayD2Snapshot sensors_ = {0, 0, 0, 0, 0, false, 0, 0};
  uint16_t airspeed_ = 0;
  uint16_t rollAlarm_ = 0;
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
float g_prevNeedleSpeed = -1.0f;

inline float speedToAngleDeg(float speed) {
  return kGaugeStartDeg + (speed / kGaugeMaxSpeed) * kGaugeSweepDeg;
}

inline int16_t gx(float angleDeg, float r) {
  return kGaugeCx + static_cast<int16_t>(r * sinf(angleDeg * PI / 180.0f));
}

inline int16_t gy(float angleDeg, float r) {
  return kGaugeCy - static_cast<int16_t>(r * cosf(angleDeg * PI / 180.0f));
}

void drawArcBand(float startSpeed, float endSpeed, uint16_t color) {
  float a0 = speedToAngleDeg(startSpeed);
  float a1 = speedToAngleDeg(endSpeed);
  for (float a = a0; a <= a1; a += 0.5f) {
    g_tft.drawLine(gx(a, kArcInnerR), gy(a, kArcInnerR),
                   gx(a, kArcOuterR), gy(a, kArcOuterR), color);
  }
}

void drawNeedle(float speed, uint16_t color) {
  float clamped = speed < 0.0f ? 0.0f : (speed > kGaugeMaxSpeed ? kGaugeMaxSpeed : speed);
  float rad = speedToAngleDeg(clamped) * PI / 180.0f;
  int16_t tx = kGaugeCx + static_cast<int16_t>(kNeedleLen * sinf(rad));
  int16_t ty = kGaugeCy - static_cast<int16_t>(kNeedleLen * cosf(rad));
  float perpRad = rad + PI / 2.0f;
  int16_t bx1 = kGaugeCx + static_cast<int16_t>(3 * sinf(perpRad));
  int16_t by1 = kGaugeCy - static_cast<int16_t>(3 * cosf(perpRad));
  int16_t bx2 = kGaugeCx - static_cast<int16_t>(3 * sinf(perpRad));
  int16_t by2 = kGaugeCy + static_cast<int16_t>(3 * cosf(perpRad));
  g_tft.fillTriangle(tx, ty, bx1, by1, bx2, by2, color);
}

void drawGaugeFrame() {
  g_tft.drawCircle(kGaugeCx, kGaugeCy, kGaugeOuterR, TFT_WHITE);
  g_tft.drawCircle(kGaugeCx, kGaugeCy, kGaugeOuterR - 1, TFT_WHITE);

  drawArcBand(2.0f, 5.0f, TFT_RED);
  drawArcBand(6.0f, 7.0f, TFT_GREEN);
  drawArcBand(8.0f, 10.0f, TFT_RED);

  g_tft.loadFont(Arial_Black22);
  g_tft.setTextColor(TFT_WHITE, kDisplayBackground);
  for (int i = 0; i <= 10; i++) {
    float angle = speedToAngleDeg(static_cast<float>(i));
    g_tft.drawLine(gx(angle, kTickOuterR), gy(angle, kTickOuterR),
                   gx(angle, kTickMajorInnerR), gy(angle, kTickMajorInnerR), TFT_WHITE);
    char num[4];
    snprintf(num, sizeof(num), "%d", i);
    g_tft.drawCentreString(num, gx(angle, kNumberR), gy(angle, kNumberR) - 11, 1);
  }
  g_tft.unloadFont();

  for (int i = 1; i < 20; i += 2) {
    float angle = speedToAngleDeg(i * 0.5f);
    g_tft.drawLine(gx(angle, kTickOuterR), gy(angle, kTickOuterR),
                   gx(angle, kTickMinorInnerR), gy(angle, kTickMinorInnerR), TFT_WHITE);
  }

  g_tft.fillCircle(kGaugeCx, kGaugeCy, 5, TFT_WHITE);
}

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

  // roll alarm をdisplay_d2 に送信（断線時は無視）
  Wire.beginTransmission(kDisplayD2I2CAddress);
  Wire.write(static_cast<uint8_t>(g_slave.rollAlarm()));
  Wire.endTransmission();

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
  if (millis() - g_lastRenderAt < kDisplayRefreshIntervalMs) return;
  g_lastRenderAt = millis();

  float currentSpeed = g_slave.airspeed() / 10.0f;

  // ── Needle update (skip if unchanged) ──
  float newAngle = speedToAngleDeg(
    currentSpeed < 0.0f ? 0.0f : (currentSpeed > kGaugeMaxSpeed ? kGaugeMaxSpeed : currentSpeed));
  bool needleChanged = (g_prevNeedleSpeed < 0.0f) ||
    (fabsf(newAngle - speedToAngleDeg(g_prevNeedleSpeed)) > 0.5f);

  if (needleChanged) {
    if (g_prevNeedleSpeed >= 0.0f) {
      drawNeedle(g_prevNeedleSpeed, kDisplayBackground);
    }
    drawArcBand(2.0f, 5.0f, TFT_RED);
    drawArcBand(6.0f, 7.0f, TFT_GREEN);
    drawArcBand(8.0f, 10.0f, TFT_RED);
    g_tft.fillCircle(kGaugeCx, kGaugeCy, 5, TFT_WHITE);
    drawNeedle(currentSpeed, TFT_RED);
    g_prevNeedleSpeed = currentSpeed;
  }

  g_slave.task();

  // ── Center digital speed (Arial_Black56) ──
  g_tft.loadFont(Arial_Black56);
  g_tft.setTextColor(TFT_WHITE, kDisplayBackground);
  g_tft.setTextPadding(g_tft.textWidth("00.0"));
  char speedStr[8];
  int whole = static_cast<int>(currentSpeed);
  int frac = static_cast<int>(currentSpeed * 10.0f) % 10;
  snprintf(speedStr, sizeof(speedStr), "%d.%d", whole, frac);
  g_tft.drawCentreString(speedStr, kGaugeCx, kGaugeCy - 28, 1);
  g_tft.setTextPadding(0);
  g_tft.unloadFont();

  g_slave.task();

  // ── Bottom info panel (built-in font 2) ──
  g_tft.setTextColor(TFT_WHITE, kDisplayBackground);
  char line[42];
  int y = 250;

  snprintf(line, sizeof(line), "B:%-5u Bt:%-5u",
           g_snapshot.baroAlt, g_snapshot.batteryVoltage);
  g_tft.drawString(line, 4, y, 2);
  y += 18;

  snprintf(line, sizeof(line), "P1:%-4u P2:%-4u U:%-4u",
           g_snapshot.potentiometer1, g_snapshot.potentiometer2, g_snapshot.ultrasonicAlt);
  g_tft.drawString(line, 4, y, 2);
  y += 18;

  g_slave.task();

  snprintf(line, sizeof(line), "I2C:%-3s OK:%-5lu E:%-5lu",
           g_snapshot.connected ? "OK" : "ERR",
           static_cast<unsigned long>(g_snapshot.successCount),
           static_cast<unsigned long>(g_snapshot.failureCount));
  g_tft.setTextColor(g_snapshot.connected ? TFT_GREEN : TFT_RED, kDisplayBackground);
  g_tft.drawString(line, 4, y, 2);
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

  Wire.begin(6, 7);  // SDA=GPIO6, SCL=GPIO7 (XIAO ESP32C3)
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
  drawGaugeFrame();

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
