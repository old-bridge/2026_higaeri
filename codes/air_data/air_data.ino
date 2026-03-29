#include <HardwareSerial.h>

#include "ModbusConfig.h"
#include "ModbusSlave.h"

constexpr uint8_t kRs485DePin = D10;
constexpr uint8_t kModbusRxPin = D7;  // GPIO20 (UART0 default RX)
constexpr uint8_t kModbusTxPin = D6;  // GPIO21 (UART0 default TX)
constexpr uint8_t kStatusLedPin = D0;
constexpr uint8_t kEncoderPin = D8;
constexpr uint8_t kBatterySensePin = A0;

// Airspeed is derived from the pulse count measured in each 0.5 s window.
constexpr uint32_t kAirspeedSampleWindowMs = 500;
constexpr float kAirspeedSlope = 1/300.0f;  // 仮置き
constexpr float kAirspeedOffset = 0.0f;
constexpr uint32_t kDebugPrintIntervalMs = 1000;


struct AirDataSnapshot {
  uint16_t windSpeed;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
};

class AirDataSlaveNode : public ModbusSlaveBase {
public:
  explicit AirDataSlaveNode(HardwareSerial& serial)
    : ModbusSlaveBase(serial, kAirDataSlaveId, kRs485DePin, kModbusRxPin, kModbusTxPin) {}

  void setSnapshot(const AirDataSnapshot& snapshot) {
    snapshot_ = snapshot;
  }

  void task() {
    setHoldingValue(kAirWindSpeedRegister, snapshot_.windSpeed);
    setHoldingValue(kAirAs5600PrimaryRegister, snapshot_.as5600Primary);
    setHoldingValue(kAirAs5600SecondaryRegister, snapshot_.as5600Secondary);
    setHoldingValue(kAirBatteryRegister, snapshot_.batteryRaw);
    ModbusSlaveBase::task();
  }

protected:
  void setupRegisters() override {
    addHoldingRegisters(kAirDataReadStart, kAirDataReadCount, 0);
    addHoldingRegisters(kAirDataWriteStart, kAirDataWriteCount, 0);
  }

  void setupCallbacks() override {
    registerReadHandler(kAirDataReadStart, kAirDataReadCount);
    registerWriteHandler(kAirDataWriteStart, kAirDataWriteCount);
  }

  uint16_t onWriteRegister(TRegister* reg, uint16_t newValue) override {
    if (reg->address.address != kAirDataWriteStart) {
      return newValue;
    }

    if (newValue == kAirCommandLedOn) {
      digitalWrite(kStatusLedPin, HIGH);
    } else if (newValue == kAirCommandLedOff) {
      digitalWrite(kStatusLedPin, LOW);
    }

    return kAirCommandNone;
  }

private:
  AirDataSnapshot snapshot_ = {0, 0, 0, 0};
};

namespace {
HardwareSerial MySerial0(0);
AirDataSlaveNode g_slave(MySerial0);
volatile uint32_t g_encoderPulseCount = 0;
AirDataSnapshot g_snapshot = {0, 0, 0, 0};
uint32_t g_lastSamplePulseCount = 0;
unsigned long g_lastAirspeedSampleAt = 0;
unsigned long g_lastDebugAt = 0;

void IRAM_ATTR handleEncoderPulse() {
  g_encoderPulseCount++;
}

uint16_t convertPulseCountToWindSpeed(uint32_t pulseCountInWindow) {
  // Replace this linear model once the actual calibration formula is fixed.
  const float windSpeed = (kAirspeedSlope * static_cast<float>(pulseCountInWindow)) + kAirspeedOffset;
  

  if (windSpeed <= 0.0f) {
    return 0;
  }
  
  return static_cast<uint16_t>(windSpeed);
}

void updateWindSpeed() {
  if (millis() - g_lastAirspeedSampleAt < kAirspeedSampleWindowMs) {
    return;
  }

  const uint32_t currentPulseCount = g_encoderPulseCount;
  const uint32_t pulseCountInWindow = currentPulseCount - g_lastSamplePulseCount;

  g_snapshot.windSpeed = convertPulseCountToWindSpeed(pulseCountInWindow);
  g_lastSamplePulseCount = currentPulseCount;
  g_lastAirspeedSampleAt = millis();
}

void updateSensors() {
  updateWindSpeed();
  g_snapshot.as5600Primary = 0;
  g_snapshot.as5600Secondary = 0;
  g_snapshot.batteryRaw = analogRead(kBatterySensePin);
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }
  g_lastDebugAt = millis();
  Serial.printf("[air_data] windSpeed=%u  pulseTotal=%lu  battRaw=%u  as5600p=%u  as5600s=%u\n",
    g_snapshot.windSpeed,
    static_cast<unsigned long>(g_encoderPulseCount),
    g_snapshot.batteryRaw,
    g_snapshot.as5600Primary,
    g_snapshot.as5600Secondary);
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(100);

  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);
  pinMode(kEncoderPin, INPUT_PULLUP);
  pinMode(kBatterySensePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kEncoderPin), handleEncoderPulse, RISING);
  g_lastAirspeedSampleAt = millis();

  g_slave.begin();
}

void loop() {
  updateSensors();
  g_slave.setSnapshot(g_snapshot);
  g_slave.task();
  printDebug();
}
