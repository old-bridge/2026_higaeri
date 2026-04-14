#include <esp_now.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>

#include "ModbusConfig.h"
#include "ModbusSlave.h"

constexpr uint8_t kRs485DePin = D10;
constexpr uint8_t kModbusRxPin = D7;  // GPIO20 (UART0 default RX)
constexpr uint8_t kModbusTxPin = D6;  // GPIO21 (UART0 default TX)
constexpr uint8_t kStatusLedPin = D0;
constexpr uint8_t kEncoderPin = D8;
constexpr uint8_t kBatterySensePin = A0;
constexpr uint8_t kAosSdaPin = D2;   // ソフトウェアI2C SDA (AoS)
constexpr uint8_t kAosSclPin = D3;   // ソフトウェアI2C SCL (AoS)

constexpr uint8_t  kAs5600Address  = 0x36;
constexpr uint8_t  kAs5600RegAngle = 0x0C;

// Airspeed is derived from the pulse count per second measured over a ~0.5 s window.
constexpr uint32_t kAirspeedSampleWindowMs = 500;
constexpr float kAirspeedSlope = 1/300.0f;  // 仮置き
constexpr float kAirspeedOffset = 0.0f;
constexpr uint32_t kDebugPrintIntervalMs = 1000;
constexpr uint32_t kEspNowSendIntervalMs = 500;

struct EspNowAirDataPacket {
  uint8_t  deviceId;        // 0x01 = air_data
  uint8_t  reserved;
  uint16_t windSpeed;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};


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
unsigned long g_lastEspNowSendAt = 0;
uint32_t g_espNowSequence = 0;
const uint8_t kBroadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void sendEspNow() {
  if (millis() - g_lastEspNowSendAt < kEspNowSendIntervalMs) {
    return;
  }
  g_lastEspNowSendAt = millis();

  EspNowAirDataPacket packet;
  packet.deviceId        = 0x01;
  packet.reserved        = 0;
  packet.windSpeed       = g_snapshot.windSpeed;
  packet.as5600Primary   = g_snapshot.as5600Primary;
  packet.as5600Secondary = g_snapshot.as5600Secondary;
  packet.batteryRaw      = g_snapshot.batteryRaw;
  packet.sequenceNumber  = ++g_espNowSequence;

  esp_now_send(kBroadcastAddress, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void IRAM_ATTR handleEncoderPulse() {
  g_encoderPulseCount++;
}

uint16_t convertPulseCountToWindSpeed(uint32_t pulseCountInWindow, float elapsedSec) {
  // Replace this linear model once the actual calibration formula is fixed.
  const float pulsesPerSec = static_cast<float>(pulseCountInWindow) / elapsedSec;
  const float windSpeed = (kAirspeedSlope * pulsesPerSec) + kAirspeedOffset;

  if (windSpeed <= 0.0f) {
    return 0;
  }

  return static_cast<uint16_t>(windSpeed);
}

void updateWindSpeed() {
  const unsigned long nowUs = micros();
  if (nowUs - g_lastAirspeedSampleAt < kAirspeedSampleWindowMs * 1000UL) {
    return;
  }

  const uint32_t currentPulseCount = g_encoderPulseCount;
  const uint32_t pulseCountInWindow = currentPulseCount - g_lastSamplePulseCount;
  const float elapsedSec = (nowUs - g_lastAirspeedSampleAt) / 1e6f;

  g_snapshot.windSpeed = convertPulseCountToWindSpeed(pulseCountInWindow, elapsedSec);
  g_lastSamplePulseCount = currentPulseCount;
  g_lastAirspeedSampleAt = nowUs;
}

uint16_t readAS5600(TwoWire& wire) {
  wire.beginTransmission(kAs5600Address);
  wire.write(kAs5600RegAngle);
  wire.endTransmission(false);
  wire.requestFrom(kAs5600Address, 2);
  uint16_t raw = 0;
  raw  = ((uint16_t)wire.read() << 8) & 0x0F00;
  raw |= (uint16_t)wire.read();
  return raw;
}

void updateSensors() {
  updateWindSpeed();
  uint16_t AoA = readAS5600(Wire);   // 迎え角: ハードウェアI2C
  uint16_t AoS = readAS5600(Wire1);  // 横滑り角: ソフトウェアI2C (D2/D3)
  g_snapshot.as5600Primary   = AoA;
  g_snapshot.as5600Secondary = AoS;
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
  g_lastAirspeedSampleAt = micros();

  // ハードウェアI2C (AoA: 迎え角)
  Wire.begin();
  Wire.setClock(400000);
  // ソフトウェアI2C (AoS: 横滑り角): D2=SDA, D3=SCL
  Wire1.begin(kAosSdaPin, kAosSclPin);
  Wire1.setClock(400000);

  g_slave.begin();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastAddress, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println("[air_data] ESP-NOW init OK");
  } else {
    Serial.println("[air_data] ESP-NOW init FAILED");
  }
}

void loop() {
  updateSensors();
  g_slave.setSnapshot(g_snapshot);
  g_slave.task();
  sendEspNow();
  printDebug();
}
