/**
 * air_data_test.ino
 * エアデータマイコン（Slave ID: 1）- テスト用コード
 * 
 * 機能：
 * - ランダムな値をセンサーデータとして送信（テスト用）
 * - ロータリーエンコーダ、AS5600①②、ADCの値をシミュレート
 * - Modbus通信テストに使用
 */

#define DE_PIN D10             // RS485制御ピン

#include "../shared/ModbusConfig.h"
#include "../shared/ModbusSlave.h"

// ===================================================================
// グローバル変数
// ===================================================================

// テスト用データ
volatile uint32_t testEncoderPulseCount = 0;  // テスト用パルスカウント
uint16_t testRotationCount = 0;               // テスト用回転数

// スレーブクラスの実装（テスト版）
class AirDataSlave : public ModbusSlaveBase {
private:
  // センサー値バッファ
  uint16_t sensorValues[AIR_REG_READ_SIZE];
  
  // テスト用ランダム生成パラメータ
  uint16_t randomRotation = 0;
  uint16_t randomAngle1 = 0;
  uint16_t randomAngle2 = 0;
  uint16_t randomVoltage = 0;

public:
  AirDataSlave(HardwareSerial* hwSerial, uint8_t id, uint8_t de)
    : ModbusSlaveBase(hwSerial, id, de) {
    for (int i = 0; i < AIR_REG_READ_SIZE; i++) {
      sensorValues[i] = 0;
    }
  }

protected:
  /**
   * レジスタの初期化
   */
  void setupRegisters() override {
    // 読み込み用レジスタ
    mb.addHreg(AIR_REG_READ, 100, AIR_REG_READ_SIZE);

    // 書き込み用レジスタ
    mb.addHreg(AIR_REG_WRITE, 0, AIR_REG_WRITE_SIZE);

    // ボーレート制御用レジスタ
    mb.addHreg(AIR_REG_BAUD_CTRL, 0);
  }

  /**
   * コールバック関数の登録
   */
  void setupCallbacks() override {
    // 読み込み時のコールバック
    mb.onGetHreg(AIR_REG_READ, cbRead, AIR_REG_READ_SIZE);

    // 書き込み時のコールバック
    mb.onSetHreg(AIR_REG_WRITE, cbWrite, AIR_REG_WRITE_SIZE);

    // ボーレート切り替え時のコールバック
    mb.onSetHreg(AIR_REG_BAUD_CTRL, cbWriteBaud, 1);
  }

  /**
   * 読み込みコールバック
   */
  static uint16_t cbRead(TRegister* reg, uint16_t oldValue) {
    Serial.printf("[AIR_DATA_TEST] READ offset:%d value:%d\n",
                  reg->address.address, oldValue);
    return oldValue;
  }

  /**
   * 書き込みコールバック（LED等の制御）
   */
  static uint16_t cbWrite(TRegister* reg, uint16_t newValue) {
    uint16_t offset = reg->address.address;
    Serial.printf("[AIR_DATA_TEST] WRITE offset:%d value:%d\n", offset, newValue);
    
    // 例：LED制御
    if (offset == AIR_REG_WRITE) {
      if (newValue == LED_ON) {
        Serial.println("[AIR_DATA_TEST] LED ON");
      } else {
        Serial.println("[AIR_DATA_TEST] LED OFF");
      }
    }
    
    return newValue;
  }

  /**
   * ボーレート切り替えコールバック
   */
  static uint16_t cbWriteBaud(TRegister* reg, uint16_t newValue) {
    Serial.printf("[AIR_DATA_TEST] BAUD CHANGE REQUEST: index %d\n", newValue);
    return 999;  // 確認応答
  }

public:
  /**
   * テスト用：ランダムなセンサー値を生成して更新
   */
  void updateSensorValues() {
    // ===== テスト用ランダムな値を生成 =====
    
    // sensorValues[0] - ロータリーエンコーダ（回転数）
    // 回転数: 0～1000回転の範囲でランダム
    randomRotation = random(0, 1001);
    sensorValues[0] = randomRotation;
    
    // sensorValues[1] - AS5600①（I2C角度）
    // 角度: 0～4095（12ビット）の範囲でランダム
    randomAngle1 = random(0, 4096);
    sensorValues[1] = randomAngle1;
    
    // sensorValues[2] - AS5600②（I2C角度）
    // 角度: 0～4095（12ビット）の範囲でランダム
    randomAngle2 = random(0, 4096);
    sensorValues[2] = randomAngle2;
    
    // sensorValues[3] - ADC（バッテリー電圧）
    // 電圧: 10V～15Vの範囲でランダム（100倍スケール: 1000～1500）
    randomVoltage = random(1000, 1501);
    sensorValues[3] = randomVoltage;

    // Modbusレジスタに書き込み
    for (int i = 0; i < AIR_REG_READ_SIZE; i++) {
      mb.Hreg(AIR_REG_READ + i, sensorValues[i]);
    }
  }
  
  /**
   * テスト用：現在の値をシリアルに出力
   */
  void printTestValues() {
    Serial.printf("[TEST_VALUES] Rotation:%u, Angle1:%u, Angle2:%u, Voltage:%u\n",
                  randomRotation, randomAngle1, randomAngle2, randomVoltage);
  }
};

// グローバルインスタンス
AirDataSlave airDataSlave(&Serial0, SLAVE_ID_AIR_DATA, DE_PIN);

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // 乱数生成のシード設定
  randomSeed(analogRead(A0));
  
  airDataSlave.begin();
  
  Serial.println("");
  Serial.println("=================================================");
  Serial.println("[AIR_DATA_TEST] Initialized - Running TEST MODE");
  Serial.println("=================================================");
  Serial.println("Generating random sensor values:");
  Serial.println("  - Rotation: 0～1000 rotations");
  Serial.println("  - Angle1:   0～4095 (12-bit)");
  Serial.println("  - Angle2:   0～4095 (12-bit)");
  Serial.println("  - Voltage:  1000～1500 (10V～15V, 100x scale)");
  Serial.println("=================================================");
  Serial.println("");
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // ランダムなセンサー値を更新
  airDataSlave.updateSensorValues();
  
  // Modbusタスク実行
  airDataSlave.task();
  
  // デバッグ出力（1秒ごと）
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000) {
    lastDebugTime = millis();
    airDataSlave.printTestValues();
  }
}
