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

// 公式ドキュメントに基づく USBシリアル + UART0/UART1 同時使用
// https://wiki.seeedstudio.com/xiao_esp32c3_getting_started/
#include <HardwareSerial.h>

// Define hardware serial mapped to internal UARTs
HardwareSerial MySerial0(0);  // UART0 - Modbus RS485イン
HardwareSerial MySerial1(1);  // UART1 - 予約（将来拡張用）

#include "pins.h"

#define ENCODER_PIN D1         // ロータリーエンコーダのパルスピン
#define ENCODER_PULSES_PER_REV 100  // 100パルス/1回転

#include "ModbusConfig.h"
#include "ModbusSlave.h"
#include "ModbusMaster.h"
// Note: ModbusSlave.cpp and ModbusMaster.cpp are compiled as separate units
// by the Arduino IDE, so we don't need to include them here

// ===================================================================
// グローバル変数
// ===================================================================

// ロータリーエンコーダの状態
volatile uint32_t encoderPulseCount = 0;  // パルスカウント
uint32_t lastEncoderPulseCount = 0;       // 前回読み込み時のパルスカウント
uint16_t rotationCount = 0;               // 回転数（パルス数 / 100）
const int LED_PIN = D0;

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
   * ModbusRTUServer では自動的にレジスタバッファを管理します
   * INPUT_REGISTER_NUM と HOLDING_REGISTER_NUM で定義されたサイズが使用されます
   * setupRegisters() は不要 - 削除
   */
  void setupRegisters() override {
    // ModbusRTUServer は自動的に INPUT_REGISTER_NUM / HOLDING_REGISTER_NUM 
    // サイズのレジスタバッファを持っています
    // ここでの初期化は不要です
  }

  /**
   * イベントハンドラーの登録
   * ModbusRTUServer では、setWriteHoldingRegisterEvent() でイベントを設定
   */
  void setupCallbacks() override {
    // イベントハンドラーを登録（必要に応じて）
    // mb.setReadHoldingRegistersEvent(eventHandler, context);
    // mb.setWriteHoldingRegisterEvent(eventHandler, context);
  }

public:
  /**
   * センサー値を更新し、Modbusレジスタに書き込み
   * loop()内から呼び出し
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

    // ModbusRTUServer のインプットレジスタに書き込み
    // (Modbusマスターが読み取るための読み込み専用データ)
    for (int i = 0; i < AIR_REG_READ_SIZE; i++) {
      mb.setInputValue(AIR_REG_READ + i, sensorValues[i]);
    }
  }
  
  /**
   * Modbusレジスタからコマンド値を読み込み
   * writeコマンドが受け取られたかチェック
   * 
   * @return true if write command was received
   */
  bool checkCommands() {
    // ホールディングレジスタ (AIR_REG_WRITE) をチェック
    uint16_t cmdValue = mb.getHoldingValue(AIR_REG_WRITE);
    
    if (cmdValue != 0) {
      Serial.printf("[AIR_DATA] Received command: %d\n", cmdValue);
      
      // LED制御など
      if (cmdValue == LED_ON) {
        Serial.println("[AIR_DATA] LED ON");
      } else if (cmdValue == LED_OFF) {
        Serial.println("[AIR_DATA] LED OFF");
      }
      
      // コマンド消去
      mb.setHoldingValue(AIR_REG_WRITE, 0);
      return true;
    }
    
    // ボーレート変更コマンドをチェック
    uint16_t baudCmd = mb.getHoldingValue(AIR_REG_BAUD_CTRL);
    if (baudCmd != 0 && baudCmd < NUM_BAUDS) {
      Serial.printf("[AIR_DATA] Baud change requested: %d\n", baudCmd);
      changeBaud(baudCmd);
      // コマンド消去
      mb.setHoldingValue(AIR_REG_BAUD_CTRL, 0);
      return true;
    }
    
    return false;
  }
};

// グローバルインスタンス
// MySerial0 は air_data.ino グローバルスコープで定義
AirDataSlave airDataSlave(&MySerial0, SLAVE_ID_AIR_DATA, DE_PIN);

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
  // ==== USB Serial （シリアルモニター用）====
  Serial.begin(115200);
  delay(100);  // USB CDCの初期化を待つ
  Serial.println("\n\n========== air_data.ino starting ==========");
  
  // ==== MySerial0 （Modbus RS485用 - UART0）====
  // RX: D7 (GPIO5), TX: D6 (GPIO4) - XIAO ESP32C3 の UART0 デフォルトピン
  MySerial0.begin(9600, SERIAL_8N1, 5, 4);
  Serial.println("[SETUP] MySerial0 initialized on UART0 (9600 baud)");
  
  // ==== MySerial1 （予約 - UART1）====
  // RX: D9 (GPIO9), TX: D10 (GPIO8)
  // MySerial1.begin(115200, SERIAL_8N1, 9, 10);
  // Serial.println("[SETUP] MySerial1 initialized on UART1 (115200 baud)");
  
  // ロータリーエンコーダの初期化
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
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
