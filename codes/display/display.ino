/**
 * display.ino
 * 表示マイコン（Slave ID: 2）
 * 
 * 機能：
 * - TFTディスプレイにアラー・ロガーのデータを表示
 * - ポテンショメータ①②から角度を読む（ADC）
 * - バッテリー電圧を監視（ADC）
 * - 超音波センサから高度を読む（Soft UART）
 * - 大気圧センサから高度を読む（I2C）
 * - TFT_eSPIライブラリで表示制御
 */

#define DE_PIN D3  // RS485制御ピン

#include "../shared/ModbusConfig.h"
#include "../shared/ModbusSlave.h"
#include "../shared/TFTDisplay.h"
#include "User_Setup.h"  // TFT_eSPI設定ファイル

// ===================================================================
// グローバル変数
// ===================================================================

// スレーブクラスの実装
class DisplaySlave : public ModbusSlaveBase {
private:
  // センサー値バッファ
  uint16_t sensorValues[DISP_REG_READ_SIZE];
  
  // 実装予定のセンサー関連
  // - ポテンショメータ①②（ADC）
  // - バッテリー電圧（ADC）
  // - 超音波センサ（Soft UART）
  // - 大気圧センサ（I2C）

public:
  DisplaySlave(HardwareSerial* hwSerial, uint8_t id, uint8_t de)
    : ModbusSlaveBase(hwSerial, id, de) {
    for (int i = 0; i < DISP_REG_READ_SIZE; i++) {
      sensorValues[i] = 0;
    }
  }

protected:
  /**
   * レジスタの初期化
   */
  void setupRegisters() override {
    // 読み込み用レジスタ
    mb.addHreg(DISP_REG_READ, 0, DISP_REG_READ_SIZE);

    // 書き込み用レジスタ
    mb.addHreg(DISP_REG_WRITE, 0, DISP_REG_WRITE_SIZE);

    // ボーレート制御用レジスタ
    mb.addHreg(DISP_REG_BAUD_CTRL, 0);
  }

  /**
   * コールバック関数の登録
   */
  void setupCallbacks() override {
    // 読み込み時のコールバック
    mb.onGetHreg(DISP_REG_READ, cbRead, DISP_REG_READ_SIZE);

    // 書き込み時のコールバック
    mb.onSetHreg(DISP_REG_WRITE, cbWrite, DISP_REG_WRITE_SIZE);

    // ボーレート切り替え時のコールバック
    mb.onSetHreg(DISP_REG_BAUD_CTRL, cbWriteBaud, 1);
  }

  /**
   * 読み込みコールバック
   */
  static uint16_t cbRead(TRegister* reg, uint16_t oldValue) {
    Serial.printf("[DISPLAY] READ offset:%d value:%d @ %ld bps\n",
                  reg->address.address, oldValue, BAUDRATES[0]);
    return oldValue;
  }

  /**
   * 書き込みコールバック（LED等の制御）
   */
  static uint16_t cbWrite(TRegister* reg, uint16_t newValue) {
    uint16_t offset = reg->address.address;
    Serial.printf("[DISPLAY] WRITE offset:%d value:%d\n", offset, newValue);
    
    // 例：LED制御
    if (offset == DISP_REG_WRITE) {
      if (newValue == LED_ON) {
        // digitalWrite(LED_PIN, HIGH);
        Serial.println("[DISPLAY] LED ON");
      } else {
        // digitalWrite(LED_PIN, LOW);
        Serial.println("[DISPLAY] LED OFF");
      }
    }
    
    return newValue;
  }

  /**
   * ボーレート切り替えコールバック
   */
  static uint16_t cbWriteBaud(TRegister* reg, uint16_t newValue) {
    Serial.printf("[DISPLAY] BAUD CHANGE REQUEST: index %d\n", newValue);
    // changeBaud()は別途呼び出し必要
    return 999;  // 確認応答
  }

public:
  /**
   * センサー値を更新
   */
  void updateSensorValues() {
    // TODO: 実装
    // sensorValues[0] = readPotentiometer1();
    // sensorValues[1] = readPotentiometer2();
    // sensorValues[2] = readBatteryVoltage();
    // sensorValues[3] = readUltrasonicAltitude();
    // sensorValues[4] = readPressureAltitude();

    // 仮の値（テスト用）
    for (int i = 0; i < DISP_REG_READ_SIZE; i++) {
      mb.Hreg(DISP_REG_READ + i, 200 + i * 20);
    }
  }
};

// グローバルインスタンス
DisplaySlave displaySlave(&Serial0, SLAVE_ID_DISPLAY, DE_PIN);
DisplayManager display;

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  displaySlave.begin();
  display.begin();
  
  // TODO: センサーの初期化
  // initPotentiometer();
  // initADC();
  // initUltrasonicSensor();
  // initPressureSensor();
  
  // 初期表示
  display.clear();
  display.drawText(10, 10, "Display PCB Init OK", 2, TFT_GREEN);
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // センサー値を更新
  displaySlave.updateSensorValues();
  
  // Modbusタスク実行
  displaySlave.task();
  
  // TFT表示更新
  updateDisplay();
  
  // TODO: ボーレート切り替えリクエストの監視など
}

// ===================================================================
// 表示更新関数
// ===================================================================

/**
 * TFT画面を更新
 * 現状は回転数と高度のみを表示
 */
void updateDisplay() {
  static unsigned long lastUpdateTime = 0;
  const unsigned long UPDATE_INTERVAL = 100;  // 100ms ごと
  
  if (millis() - lastUpdateTime < UPDATE_INTERVAL) {
    return;
  }
  lastUpdateTime = millis();
  
  // 画面クリア
  display.clear();
  
  // ヘッダ表示
  display.drawText(10, 10, "Flight Data", 2, TFT_YELLOW);
  
  // TODO: 実際の表示内容を追加
  // - 回転数（エアデータからの値）
  // - 高度（複数のセンサーからの値）
  // - その他センサー値
  
  // 仮の表示
  display.drawText(10, 50, "Rotation: N/A", 2, TFT_WHITE);
  display.drawText(10, 80, "Altitude: N/A", 2, TFT_WHITE);
}

/**
 * ロガーから受け取ったデータで画面を更新する場合
 * 別途ロガーから値を受け取る仕組みが必要
 */
void displayAirDataValues(uint16_t rotation, uint16_t as56001, uint16_t as56002, uint16_t battery) {
  // TODO: 実装
  // 画面レイアウト案：
  // +---------------------+
  // | Flight Data         |
  // +---------+-----------+
  // | Rot: 1234 | Spd: 100 |
  // | Alt: 5000 |  Tilt: 45|
  // +---------------------+
}
