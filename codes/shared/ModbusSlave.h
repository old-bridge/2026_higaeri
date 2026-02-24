#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <ModbusRTU.h>
#include "ModbusConfig.h"

/**
 * ModbusSlaveBase
 * スレーブマイコン用の基本クラス
 * 各スレーブはこのクラスを継承して具体的な処理を実装
 */
class ModbusSlaveBase {
protected:
  ModbusRTU mb;
  HardwareSerial* serial;
  uint8_t slaveId;
  uint8_t dePin;
  int currentBaudIdx;

  // 各スレーブが実装すべき仮想メソッド
  virtual void setupRegisters() = 0;
  virtual void setupCallbacks() = 0;

public:
  ModbusSlaveBase(HardwareSerial* hwSerial, uint8_t id, uint8_t de)
    : serial(hwSerial), slaveId(id), dePin(de), currentBaudIdx(0) {}

  /**
   * 初期化
   * setup()内から呼び出すこと
   */
  void begin() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);

    pinMode(dePin, OUTPUT);
    digitalWrite(dePin, LOW);

    // 9600 bauで開始
    serial->begin(9600, SERIAL_8N1, -1, -1);
    mb.begin(serial, dePin);
    mb.slave(slaveId);

    // 各スレーブ固有の初期化
    setupRegisters();
    setupCallbacks();

    Serial.printf("[SLAVE %d] READY @ 9600 baud\n", slaveId);
  }

  /**
   * Modbusタスク実行
   * loop()内から毎フレーム呼び出すこと
   */
  void task() {
    mb.task();
  }

  /**
   * ボーレート切り替え
   * REG_BAUD_CTRLへの書き込みコールバックから呼ばれる
   */
  void changeBaud(int baudIdx) {
    if (baudIdx < 0 || baudIdx >= NUM_BAUDS) {
      Serial.printf("[SLAVE %d] Invalid baud index: %d\n", slaveId, baudIdx);
      return;
    }

    if (baudIdx == currentBaudIdx) {
      return;  // 同じボーレート指定は無視
    }

    long newBaud = BAUDRATES[baudIdx];
    Serial.printf("[SLAVE %d] Switching to %ld baud...\n", slaveId, newBaud);
    Serial.flush();

    serial->flush();
    serial->end();
    delay(200);
    serial->begin(newBaud, SERIAL_8N1, -1, -1);
    delay(200);

    // トランシーバーリセット
    digitalWrite(dePin, HIGH);
    delay(5);
    digitalWrite(dePin, LOW);

    currentBaudIdx = baudIdx;
    Serial.printf("[SLAVE %d] Now at %ld baud\n\n", slaveId, newBaud);
  }

  /**
   * 現在のボーレート取得
   */
  long getCurrentBaud() const {
    return BAUDRATES[currentBaudIdx];
  }

  /**
   * 現在のボーレートインデックス取得
   */
  int getCurrentBaudIdx() const {
    return currentBaudIdx;
  }

  /**
   * レジスタに値を書き込む
   */
  void setRegisterValue(uint16_t address, uint16_t value) {
    mb.Hreg(address, value);
  }

  /**
   * レジスタの値を読み込む
   */
  uint16_t getRegisterValue(uint16_t address) const {
    return mb.Hreg(address);
  }
};

#endif  // MODBUS_SLAVE_H
