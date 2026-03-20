/**
 * display_d1.ino
 * 表示マイコン D-1（Slave ID: 2）
 * 
 * 機能：
 * - TFTディスプレイに全データを表示
 * - 大気圧センサから高度を読む（I2C）
 * - Display D-2 から他のセンサー値をI2C経由で受け取る
 * - Modbusマスター（Logger）からのリクエストに応答
 * - TFT_eSPIライブラリで表示制御
 */

#define DE_PIN D3  // RS485制御ピン

#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>  // Hardware-specific library
#include "ModbusConfig.h"
#include "ModbusSlave.h"
#include "ModbusMaster.h"
// Note: ModbusSlave.cpp and ModbusMaster.cpp are compiled as separate units
// by the Arduino IDE, so we don't need to include them here
// #include "ModbusSlave.cpp"
// #include "ModbusMaster.cpp"

#define TFT_GREY 0x5AEB

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

// ===================================================================
// I2C アドレス定義
// ===================================================================
#define I2C_ADDR_D2        0x30  // Display D-2のI2Cスレーブアドレス
#define I2C_ADDR_BARO      0x77  // 大気圧センサ（例：BMP390）のアドレス

// ===================================================================
// グローバル変数（前方宣言）
// ===================================================================

// ロータリーエンコーダ用のグローバル変数
volatile uint16_t displayedRotationCount = 0;  // 表示用の回転数

// TFTディスプレイ更新用タイマー
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;  // 100ms ごとに更新

// スレーブクラスの実装
class DisplayD1Slave : public ModbusSlaveBase {
private:
  // センサー値バッファ
  uint16_t sensorValues[DISP_D1_REG_READ_SIZE];
  
  // D-2からのセンサーデータ（I2C経由）
  struct SensorDataD2 {
    uint16_t potentiometer1;  // ポテンショメータ①
    uint16_t potentiometer2;  // ポテンショメータ②
    uint16_t batteryVoltage;  // バッテリー電圧
    uint16_t ultrasonicAlt;   // 超音波高度
  } d2Data;

  // ロータリーエンコーダの回転数（loggerから受け取り）
  uint16_t rotationCount = 0;

  // I2Cバッファ
  uint8_t i2cBuffer[8];

public:
  DisplayD1Slave(HardwareSerial* hwSerial, uint8_t id, uint8_t de)
    : ModbusSlaveBase(hwSerial, id, de) {
    for (int i = 0; i < DISP_D1_REG_READ_SIZE; i++) {
      sensorValues[i] = 0;
    }
    memset(&d2Data, 0, sizeof(d2Data));
    memset(i2cBuffer, 0, sizeof(i2cBuffer));
  }

protected:
  /**
   * レジスタの初期化
   * モダンなModbusRTUServerでは、レジスタのセットアップは不要
   * 必要に応じてスレッドセーフなアクセスのみ使用
   */
  void setupRegisters() override {
    // ModbusRTUServer では明示的なレジスタ登録は不要
    // setHoldingValue() と getHoldingValue() で直接アクセス可能
  }

  /**
   * コールバック関数の登録
   * モダンなModbusRTUServerではコールバックが不要
   */
  void setupCallbacks() override {
    // ModbusRTUServer はイベントハンドラベースのため、
    // 明示的なコールバック登録は不要
  }

public:
  /**
   * D-2からI2C経由でセンサーデータを読み込む
   */
  bool readSensorDataFromD2() {
    Wire.beginTransmission(I2C_ADDR_D2);
    Wire.write(0x00);  // レジスタアドレス 0 から読み込み開始
    if (Wire.endTransmission() != 0) {
      Serial.println("[DISPLAY_D1] I2C error: Failed to request data from D-2");
      return false;
    }

    // 8バイト読み込み（4つのuint16_t）
    int bytesRead = Wire.requestFrom(I2C_ADDR_D2, 8);
    if (bytesRead != 8) {
      Serial.printf("[DISPLAY_D1] I2C error: Expected 8 bytes, got %d\n", bytesRead);
      return false;
    }

    // バッファに読み込む
    for (int i = 0; i < 8; i++) {
      i2cBuffer[i] = Wire.read();
    }

    // uint16_t に変換（リトルエンディアン想定）
    d2Data.potentiometer1 = (i2cBuffer[1] << 8) | i2cBuffer[0];
    d2Data.potentiometer2 = (i2cBuffer[3] << 8) | i2cBuffer[2];
    d2Data.batteryVoltage = (i2cBuffer[5] << 8) | i2cBuffer[4];
    d2Data.ultrasonicAlt   = (i2cBuffer[7] << 8) | i2cBuffer[6];

    Serial.printf("[DISPLAY_D1] Received from D-2: Pot1=%d, Pot2=%d, Batt=%d, US_Alt=%d\n",
                  d2Data.potentiometer1, d2Data.potentiometer2,
                  d2Data.batteryVoltage, d2Data.ultrasonicAlt);
    return true;
  }

  /**
   * 気圧センサから高度を読み込む（スタブ）
   * 実装は使用する気圧センサライブラリに依存
   */
  bool readBarometricAltitude() {
    // TODO: 実装予定
    // BMP390またはBMP680など、使用するセンサに応じて実装
    sensorValues[0] = 0;  // ダミー値
    return true;
  }

  /**
   * Modbusレジスタを更新（新API）
   */
  void updateModbusRegisters() {
    mb.setHoldingValue(DISP_D1_REG_READ + 0, sensorValues[0]);
    mb.setHoldingValue(DISP_D1_REG_WRITE, rotationCount);  // 回転数を書き込みレジスタに設定
  }

  /**
   * 回転数を設定（ロガーからModbus経由で受け取った値）
   */
  void setRotationCount(uint16_t count) {
    rotationCount = count;
  }
  void updateTask() {
    // D-2からセンサーデータを読む
    readSensorDataFromD2();
    
    // 気圧センサを読む
    readBarometricAltitude();
    
    // Modbusレジスタを更新
    updateModbusRegisters();
    
    // グローバル回転数表示変数を更新
    displayedRotationCount = rotationCount;
  }

  // ゲッターメソッド
  uint16_t getPotentiometer1() const { return d2Data.potentiometer1; }
  uint16_t getPotentiometer2() const { return d2Data.potentiometer2; }
  uint16_t getBatteryVoltage() const { return d2Data.batteryVoltage; }
  uint16_t getUltrasonicAlt() const { return d2Data.ultrasonicAlt; }
  uint16_t getBaroAlt() const { return sensorValues[0]; }
};

// グローバルインスタンス
DisplayD1Slave displaySlave(&Serial0, SLAVE_ID_DISPLAY_3_1, DE_PIN);

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // シリアル初期化（デバッグ用）
  Serial.begin(115200);
  // タイムアウト付き待機：USB接続がない場合は5秒でスキップ
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 5000);
  delay(2000);  // Monitor initialization wait
  
  Serial.println("[DISPLAY_D1] --- System Start ---");

  Serial.println("[DISPLAY_D1] Step 1: tft.init() 実行前");
  
  tft.init();  // Initialize TFT display
  
  Serial.println("[DISPLAY_D1] Step 2: tft.init() 完了");

  tft.setRotation(0);
  tft.fillScreen(TFT_GREY);
  
  Serial.println("[DISPLAY_D1] Step 3: 画面塗りつぶし完了");

  // I2C初期化
  Wire.begin();
  Wire.setClock(400000);  // 400kHz
  Serial.println("[DISPLAY_D1] I2C initialized");

  // Modbus Slave 初期化
  displaySlave.begin();
  
  // TFT text color setup
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  
  // Draw title
  tft.drawCentreString("Display D-1", 120, 30, 4);

  Serial.println("[DISPLAY_D1] All systems initialized");
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // Modbusスレーブタスク（常に実行必要）
  displaySlave.task();

  // センサーデータ更新タスク
  displaySlave.updateTask();

  // TFTディスプレイ更新
  if (millis() - lastDisplayUpdateTime > DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateTime = millis();
    updateDisplay();
  }
}

// ===================================================================
// TFTディスプレイ更新関数
// ===================================================================
void updateDisplay() {
  // Clear screen and fill with background color
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);

  // Title
  tft.drawCentreString("=== FLIGHT DATA ===", 120, 10, 2);

  // ========== Rotation (displayed largest) ==========
  char buf[64];
  sprintf(buf, "Rotation: %d", displayedRotationCount);
  tft.drawString(buf, 10, 50, 4);  // Size 4 - large
  tft.drawString("(rpm)", 10, 100, 2);

  // Potentiometer 1, 2
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  sprintf(buf, "Pot1: %d", displaySlave.getPotentiometer1());
  tft.drawString(buf, 10, 135, 1);

  sprintf(buf, "Pot2: %d", displaySlave.getPotentiometer2());
  tft.drawString(buf, 10, 150, 1);

  // Battery Voltage
  sprintf(buf, "Battery: %dmV", displaySlave.getBatteryVoltage());
  tft.drawString(buf, 10, 170, 1);

  // Altitude information
  sprintf(buf, "US Alt: %dm", displaySlave.getUltrasonicAlt());
  tft.drawString(buf, 10, 190, 1);

  sprintf(buf, "Baro Alt: %dm", displaySlave.getBaroAlt());
  tft.drawString(buf, 10, 205, 1);

  // Status
  tft.setTextColor(TFT_GREEN, TFT_GREY);
  tft.drawString("[Slave ID: 2]", 10, 290, 1);
}
