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
#define I2C_BUFFER_SIZE 8

// ===================================================================
// グローバル変数
// ===================================================================

// センサー値バッファ
struct SensorData {
  uint16_t potentiometer1;  // ポテンショメータ①
  uint16_t potentiometer2;  // ポテンショメータ②
  uint16_t batteryVoltage;  // バッテリー電圧
  uint16_t ultrasonicAlt;   // 超音波高度
} sensorData = {0, 0, 0, 0};

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

  // バッファを送信
  Wire.write(i2cTxBuffer, I2C_BUFFER_SIZE);
  
  Serial.printf("[DISPLAY_D2] I2C data sent: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d\n",
                sensorData.potentiometer1, sensorData.potentiometer2,
                sensorData.batteryVoltage, sensorData.ultrasonicAlt);
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
 * 全センサーを読み込む
 */
void updateSensors() {
  readPotentiometers();
  readBatteryVoltage();
  readUltrasonicAltitude();

  Serial.printf("[DISPLAY_D2] Sensors: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d\n",
                sensorData.potentiometer1, sensorData.potentiometer2,
                sensorData.batteryVoltage, sensorData.ultrasonicAlt);
}

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // シリアル初期化（デバッグ用）
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

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
