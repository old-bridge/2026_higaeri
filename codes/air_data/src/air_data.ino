/**
 * air_data.ino
 * エアデータマイコン（Slave ID: 1）
 * 
 * 機能：
 * - ロータリーエンコーダから回転数を読む（100パルス/1回転）
 * - AS5600①②から角度を読む（I2C）
 * - ADCでバッテリー電圧を監視
 * - これらをModbusレジスタで提供
 * - ロガーマイコンからの制御コマンド受け取り（LED等）
 */

#define DE_PIN D10             // RS485制御ピン
#define ENCODER_PIN D1         // ロータリーエンコーダのパルスピン
#define ENCODER_PULSES_PER_REV 100  // 100パルス/1回転

#include "../shared/ModbusConfig.h"
#include "../shared/ModbusSlave.h"

// ===================================================================
// グローバル変数
// ===================================================================

// ロータリーエンコーダの状態
volatile uint32_t encoderPulseCount = 0;  // パルスカウント
uint32_t lastEncoderPulseCount = 0;       // 前回読み込み時のパルスカウント
uint16_t rotationCount = 0;               // 回転数（パルス数 / 100）

// スレーブクラスの実装
class AirDataSlave : public ModbusSlaveBase {
private:
  // センサー値バッファ
  uint16_t sensorValues[AIR_REG_READ_SIZE];
  
  // 実装予定のセンサー関連
  // sensorValues[0] - ロータリーエンコーダ（回転数）✓ 実装済み
  // sensorValues[1] - AS5600①（I2C角度）
  // sensorValues[2] - AS5600②（I2C角度）
  // sensorValues[3] - ADC（バッテリー電圧）

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
    Serial.printf("[AIR_DATA] READ offset:%d value:%d @ %ld bps\n",
                  reg->address.address, oldValue, BAUDRATES[0]);
    return oldValue;
  }

  /**
   * 書き込みコールバック（LED等の制御）
   */
  static uint16_t cbWrite(TRegister* reg, uint16_t newValue) {
    uint16_t offset = reg->address.address;
    Serial.printf("[AIR_DATA] WRITE offset:%d value:%d\n", offset, newValue);
    
    // 例：LED制御
    if (offset == AIR_REG_WRITE) {
      if (newValue == LED_ON) {
        // digitalWrite(LED_PIN, HIGH);
        Serial.println("[AIR_DATA] LED ON");
      } else {
        // digitalWrite(LED_PIN, LOW);
        Serial.println("[AIR_DATA] LED OFF");
      }
    }
    
    return newValue;
  }

  /**
   * ボーレート切り替えコールバック
   */
  static uint16_t cbWriteBaud(TRegister* reg, uint16_t newValue) {
    Serial.printf("[AIR_DATA] BAUD CHANGE REQUEST: index %d\n", newValue);
    // changeBaud()は別途呼び出し必要
    return 999;  // 確認応答
  }

public:
  /**
   * センサー値を更新
   */
  void updateSensorValues() {
    // ロータリーエンコーダの回転数を計算
    // パルス数 / 100 = 回転数
    rotationCount = (uint16_t)(encoderPulseCount / ENCODER_PULSES_PER_REV);
    sensorValues[0] = rotationCount;
    
    // TODO: 実装
    // sensorValues[1] = readAS5600_1();
    // sensorValues[2] = readAS5600_2();
    // sensorValues[3] = readBatteryVoltage();

    // Modbusレジスタに書き込み
    for (int i = 0; i < AIR_REG_READ_SIZE; i++) {
      mb.Hreg(AIR_REG_READ + i, sensorValues[i]);
    }
  }
};

// グローバルインスタンス
AirDataSlave airDataSlave(&Serial0, SLAVE_ID_AIR_DATA, DE_PIN);

// ===================================================================
// ロータリーエンコーダ割り込みハンドラ
// ===================================================================
void IRAM_ATTR encoderISR() {
  encoderPulseCount++;
}

// ===================================================================
// セットアップ
// ===================================================================
void setup() {
  // ロータリーエンコーダの初期化
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  encoderPulseCount = 0;
  
  airDataSlave.begin();
  
  // TODO: センサーの初期化
  // initAS5600();
  // initADC();
  
  Serial.printf("[AIR_DATA] Initialized. Encoder: %d pulses/rev on pin %d\n", 
                ENCODER_PULSES_PER_REV, ENCODER_PIN);
}

// ===================================================================
// ループ
// ===================================================================
void loop() {
  // センサー値を更新
  airDataSlave.updateSensorValues();
  
  // Modbusタスク実行
  airDataSlave.task();
  
  // デバッグ出力（1秒ごと）
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000) {
    lastDebugTime = millis();
    Serial.printf("[AIR_DATA] Pulses: %lu, Rotations: %u\n", 
                  encoderPulseCount, rotationCount);
  }
}
