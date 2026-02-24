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
#include "../shared/ModbusConfig.h"
#include "../shared/ModbusSlave.h"
#include "../shared/TFTDisplay.h"
#include "User_Setup.h"  // TFT_eSPI設定ファイル

// ===================================================================
// I2C アドレス定義
// ===================================================================
#define I2C_ADDR_D2        0x30  // Display D-2のI2Cスレーブアドレス
#define I2C_ADDR_BARO      0x77  // 大気圧センサ（例：BMP390）のアドレス

// ===================================================================
// グローバル変数
// ===================================================================

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
   */
  void setupRegisters() override {
    // 読み込み用レジスタ（大気圧高度のみ）
    mb.addHreg(DISP_D1_REG_READ, 0, DISP_D1_REG_READ_SIZE);

    // 書き込み用レジスタ
    mb.addHreg(DISP_D1_REG_WRITE, 0, DISP_D1_REG_WRITE_SIZE);

    // ボーレート制御用レジスタ
    mb.addHreg(DISP_D1_REG_BAUD_CTRL, 0);
  }

  /**
   * コールバック関数の登録
   */
  void setupCallbacks() override {
    // 読み込み時のコールバック
    mb.onGetHreg(DISP_D1_REG_READ, cbRead, DISP_D1_REG_READ_SIZE);

    // 書き込み時のコールバック
    mb.onSetHreg(DISP_D1_REG_WRITE, cbWrite, DISP_D1_REG_WRITE_SIZE);

    // ボーレート切り替え時のコールバック
    mb.onSetHreg(DISP_D1_REG_BAUD_CTRL, cbWriteBaud, 1);
  }

  /**
   * 読み込みコールバック
   */
  static uint16_t cbRead(TRegister* reg, uint16_t oldValue) {
    Serial.printf("[DISPLAY_D1] READ offset:%d value:%d\n",
                  reg->address.address, oldValue);
    return oldValue;
  }

  /**
   * 書き込みコールバック（LED等の制御）
   */
  static uint16_t cbWrite(TRegister* reg, uint16_t newValue) {
    uint16_t offset = reg->address.address;
    Serial.printf("[DISPLAY_D1] WRITE offset:%d value:%d\n", offset, newValue);
    
    // 例：LED制御
    if (offset == DISP_D1_REG_WRITE) {
      if (newValue == LED_ON) {
        Serial.println("[DISPLAY_D1] LED ON");
      } else {
        Serial.println("[DISPLAY_D1] LED OFF");
      }
    }
    
    return newValue;
  }

  /**
   * ボーレート切り替えコールバック
   */
  static uint16_t cbWriteBaud(TRegister* reg, uint16_t newValue) {
    // このメソッドは ModbusSlaveBase から呼ばれる
    Serial.printf("[DISPLAY_D1] Baud switch requested: index %d\n", newValue);
    return newValue;
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
   * Modbusレジスタを更新
   */
  void updateModbusRegisters() {
    mb.Hreg(DISP_D1_REG_READ + 0, sensorValues[0]);
  }

  /**
   * 定期実行タスク（loop内から呼ばれる）
   */
  void updateTask() {
    // D-2からセンサーデータを読む
    readSensorDataFromD2();
    
    // 気圧センサを読む
    readBarometricAltitude();
    
    // Modbusレジスタを更新
    updateModbusRegisters();
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
DisplayManager tftDisplay;

// TFTディスプレイ更新用タイマー
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100;  // 100ms ごとに更新

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // シリアル初期化（デバッグ用）
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("[DISPLAY_D1] Initializing...");

  // I2C初期化
  Wire.begin();
  Wire.setClock(400000);  // 400kHz
  Serial.println("[DISPLAY_D1] I2C initialized");

  // Modbus Slave 初期化
  displaySlave.begin();

  // TFT初期化
  tftDisplay.begin();
  tftDisplay.clear();
  tftDisplay.drawString("Display D-1", 10, 10, 2);

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
  // 画面をクリア（フレームごと）
  tftDisplay.clear();

  // タイトル
  tftDisplay.drawString("=== FLIGHT DATA ===", 10, 10, 2);

  // ポテンショメータ①②
  char buf[32];
  sprintf(buf, "Pot1: %d", displaySlave.getPotentiometer1());
  tftDisplay.drawString(buf, 10, 35, 1);

  sprintf(buf, "Pot2: %d", displaySlave.getPotentiometer2());
  tftDisplay.drawString(buf, 10, 50, 1);

  // バッテリー電圧
  sprintf(buf, "Battery: %dmV", displaySlave.getBatteryVoltage());
  tftDisplay.drawString(buf, 10, 70, 1);

  // 高度情報
  sprintf(buf, "US Altitude: %dm", displaySlave.getUltrasonicAlt());
  tftDisplay.drawString(buf, 10, 90, 1);

  sprintf(buf, "Baro Altitude: %dm", displaySlave.getBaroAlt());
  tftDisplay.drawString(buf, 10, 110, 1);

  // ステータス
  tftDisplay.drawString("[Modbus Slave ID: 2]", 10, 220, 1);
}
