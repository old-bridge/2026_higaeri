/**
 * display_d2.ino
 * 表示マイコン D-2（I2C Slave）
 * 
 * 機能：
 * - ポテンショメータ①②から角度を読む（ADC）
 * - バッテリー電圧を監視（ADC）
 * - 超音波センサから高度を読む（Soft UART）
 * - Display D-1 へI2C経由でデータを送信
 */

#include <Wire.h>
#include <Adafruit_DPS310.h>

// ===================================================================
// ピン定義
// ===================================================================
#define POT1_PIN          A0  // ポテンショメータ①（ADC）
#define POT2_PIN          A1  // ポテンショメータ②（ADC）
#define BATTERY_PIN       A2  // バッテリー電圧（ADC）
#define ULTRASONIC_RX_PIN D0  // 超音波センサ受信（Soft UART）
#define ULTRASONIC_TX_PIN D1  // 超音波センサ送信（Soft UART）

// ===================================================================
// I2C 設定
// ===================================================================
#define I2C_ADDR_D2  0x30  // このマイコンのI2Cスレーブアドレス
#define I2C_BUFFER_SIZE 12  // 拡張：大気圧データ用に4バイト追加

// DPS310設定
#define DPS310_I2C_ADDR 0x76  // SDOはGNDに接続
#define SEALEVELPRESSURE_HPA 1013.25  // 海面基準気圧

// ===================================================================
// グローバル変数
// ===================================================================

const int LED_PIN = D10;

// DPS310インスタンス
Adafruit_DPS310 dps;

// センサー値バッファ
struct SensorData {
  uint16_t potentiometer1;  // ポテンショメータ①
  uint16_t potentiometer2;  // ポテンショメータ②
  uint16_t batteryVoltage;  // バッテリー電圧
  uint16_t ultrasonicAlt;   // 超音波高度
  uint16_t baroAlt;         // 大気圧高度 (I2C, DPS310)
} sensorData = {0, 0, 0, 0, 0};

// I2C通信用バッファ
uint8_t i2cTxBuffer[I2C_BUFFER_SIZE] = {0};
volatile bool i2cRequestReceived = false;

// センサー読み込みタイマー
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 100;  // 100ms ごと

// ===================================================================
// I2C イベントハンドラ
// ===================================================================

/**
 * D-1からI2C受信リクエストが来たときのコールバック
 */
void onI2CRequest() {
  i2cRequestReceived = true;
  
  // センサー値をバッファに詰める（リトルエンディアン）
  i2cTxBuffer[0] = sensorData.potentiometer1 & 0xFF;
  i2cTxBuffer[1] = (sensorData.potentiometer1 >> 8) & 0xFF;
  i2cTxBuffer[2] = sensorData.potentiometer2 & 0xFF;
  i2cTxBuffer[3] = (sensorData.potentiometer2 >> 8) & 0xFF;
  i2cTxBuffer[4] = sensorData.batteryVoltage & 0xFF;
  i2cTxBuffer[5] = (sensorData.batteryVoltage >> 8) & 0xFF;
  i2cTxBuffer[6] = sensorData.ultrasonicAlt & 0xFF;
  i2cTxBuffer[7] = (sensorData.ultrasonicAlt >> 8) & 0xFF;
  i2cTxBuffer[8] = sensorData.baroAlt & 0xFF;
  i2cTxBuffer[9] = (sensorData.baroAlt >> 8) & 0xFF;
  i2cTxBuffer[10] = 0;  // 予備
  i2cTxBuffer[11] = 0;  // 予備

  // バッファを送信
  Wire.write(i2cTxBuffer, I2C_BUFFER_SIZE);
  
  Serial.printf("[DISPLAY_D2] I2C data sent: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d, Baro_Alt=%d\n",
                sensorData.potentiometer1, sensorData.potentiometer2,
                sensorData.batteryVoltage, sensorData.ultrasonicAlt, sensorData.baroAlt);
}

// ===================================================================
// センサー読み込み関数
// ===================================================================

/**
 * ポテンショメータを読み込む
 */
void readPotentiometers() {
  sensorData.potentiometer1 = analogRead(POT1_PIN);
  sensorData.potentiometer2 = analogRead(POT2_PIN);
}

/**
 * バッテリー電圧を読み込む
 */
void readBatteryVoltage() {
  // ADC値をそのまま使用（スケーリングは別途処理）
  sensorData.batteryVoltage = analogRead(BATTERY_PIN);
}

/**
 * 超音波センサから高度を読み込む（スタブ）
 * 実装は使用する超音波センサモジュールの仕様に依存
 */
void readUltrasonicAltitude() {
  // TODO: Soft UART経由でセンサデータを取得
  // 例：SoftwareSerial softSerial(ULTRASONIC_RX_PIN, ULTRASONIC_TX_PIN);
  // 使用するセンサライブラリに応じて実装
  sensorData.ultrasonicAlt = 0;  // ダミー値
}

/**
 * DPS310から大気圧高度を読み込む
 */
void readBarometricAltitude() {
  sensors_event_t temp_event, pressure_event;
  
  // データが準備できているか確認
  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    // センサからデータを取得
    if (dps.getEvents(&temp_event, &pressure_event)) {
      // 高度を計算して記録（小数点第1位までをuint16_tに変換）
      float altitude_m = dps.readAltitude(SEALEVELPRESSURE_HPA);
      sensorData.baroAlt = (uint16_t)(altitude_m * 10);  // cm単位で記録
    } else {
      sensorData.baroAlt = 0;
    }
  } else {
    sensorData.baroAlt = 0;
  }
}

/**
 * 全センサーを読み込む
 */
void updateSensors() {
  readPotentiometers();
  readBatteryVoltage();
  readUltrasonicAltitude();
  readBarometricAltitude();

  Serial.printf("[DISPLAY_D2] Sensors: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d, Baro_Alt=%d\n",
                sensorData.potentiometer1, sensorData.potentiometer2,
                sensorData.batteryVoltage, sensorData.ultrasonicAlt, sensorData.baroAlt);
}

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // シリアル初期化（デバッグ用）
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // タイムアウト付き待機：USB接続がない場合は5秒でスキップ
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 5000);
  delay(100);

  Serial.println("[DISPLAY_D2] Initializing...");

  // ピン初期化
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(ULTRASONIC_RX_PIN, INPUT);
  pinMode(ULTRASONIC_TX_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TX_PIN, LOW);

  // I2C初期化（スレーブモード）
  Wire.begin(I2C_ADDR_D2);
  Wire.onRequest(onI2CRequest);
  Serial.println("[DISPLAY_D2] I2C Slave initialized @ 0x30");

  // DPS310初期化
  if (!dps.begin_I2C(DPS310_I2C_ADDR, &Wire)) {
    Serial.println("[DISPLAY_D2] Failed to initialize DPS310");
  } else {
    Serial.println("[DISPLAY_D2] DPS310 initialized successfully");
    // 最高精度で設定
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    Serial.println("[DISPLAY_D2] DPS310 Pressure sensor ready");
  }

  Serial.println("[DISPLAY_D2] All systems initialized");
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // センサーデータ定期更新
  if (millis() - lastSensorReadTime > SENSOR_READ_INTERVAL) {
    lastSensorReadTime = millis();
    updateSensors();
  }

  // I2Cイベントは自動的に処理される（onI2CRequest）
}
