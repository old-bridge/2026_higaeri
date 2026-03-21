#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>

// XIAO ESP32C3のI2Cピン (GPIO番号)
const int I2C_SDA_PIN = 6; // XIAO D4
const int I2C_SCL_PIN = 7; // XIAO D5

Adafruit_DPS310 dps;

// 海面気圧 (hPa)
// ※より正確な高度を出したい場合は、当日の現在地の海面気圧に書き換えてください
#define SEALEVELPRESSURE_HPA (1013.25)

void setup() {
  Serial.begin(115200);
  delay(2000); // シリアルモニタ起動待ち
  Serial.println("\n--- Adafruit DPS310 Pressure & Altitude ---");

  // 1. I2Cピンを明示的に指定して初期化
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // 2. DPS310の初期化
  // SDOをGNDに落としているため、アドレスは 0x76 になります
  if (!dps.begin_I2C(0x76, &Wire)) {
    Serial.println("Failed to find DPS310! Check wiring.");
    while (1) yield();
  }
  Serial.println("DPS310 OK!");

  // 3. センサの測定精度・サンプリングレートの設定 (推奨)
  // 64回サンプリングして平均化することで、ノイズを減らし精度を上げます
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}

void loop() {
  sensors_event_t temp_event, pressure_event;

  // データが準備できているか確認
  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    // センサからデータを取得
    dps.getEvents(&temp_event, &pressure_event);

    // 気圧の表示 (hPa)
    Serial.print("Pressure: ");
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");

    // 概算高度の表示 (m)
    // readAltitude() 関数に基準となる海面気圧を渡して計算します
    Serial.print("Altitude: ");
    Serial.print(dps.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m\n");
  }

  delay(1000); // 1秒ごとに更新
}