#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <ModbusRTU.h>
#include "ModbusConfig.h"

/**
 * ModbusMaster - Modbus RTUマスター
 * ロガーマイコン用
 */
class ModbusMaster {
private:
  ModbusRTU mb;
  HardwareSerial* serial;
  uint8_t dePin;
  int currentBaudIdx;

  unsigned long lastActionTime;
  const unsigned long ACTION_INTERVAL = 5000;  // 5秒待機

public:
  ModbusMaster(HardwareSerial* hwSerial, uint8_t de)
    : serial(hwSerial), dePin(de), currentBaudIdx(0), lastActionTime(0) {}

  /**
   * 初期化
   * setup()内から呼び出すこと
   */
  void begin() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);
    Serial.println("\n=== MODBUS MASTER ===");

    pinMode(dePin, OUTPUT);
    digitalWrite(dePin, LOW);

    // 9600 bauで開始
    serial->begin(9600, SERIAL_8N1, -1, -1);
    mb.begin(serial, dePin);
    mb.master();

    Serial.println("[MASTER] READY @ 9600 baud");
  }

  /**
   * Modbusタスク実行
   * loop()内から毎フレーム呼び出すこと
   */
  void task() {
    mb.task();
  }

  /**
   * ボーレート切り替えをリクエスト
   * スレーブ側に切り替え指令を送信し、マスター側も切り替える
   * requestSlaveId: 切り替え対象スレーブのID
   * newBaudIdx: 新しいボーレートインデックス
   */
  void requestBaudChange(uint8_t requestSlaveId, int newBaudIdx) {
    if (newBaudIdx < 0 || newBaudIdx >= NUM_BAUDS) {
      Serial.printf("[MASTER] Invalid baud index: %d\n", newBaudIdx);
      return;
    }

    if (newBaudIdx == currentBaudIdx) {
      return;  // 同じボーレート指定は無視
    }

    Serial.printf("[MASTER] Requesting baud switch to index %d (%ld bps)...\n",
                  newBaudIdx, BAUDRATES[newBaudIdx]);

    // スレーブにボーレート切り替え指令を送信
    uint16_t baudIdx = (uint16_t)newBaudIdx;
    mb.writeHreg(requestSlaveId, REG_BAUD_CTRL, &baudIdx, 1);

    delay(500);  // スレーブ側の処理を待つ

    // マスター側もボーレート切り替え
    changeBaud(newBaudIdx);
  }

  /**
   * スレーブから複数レジスタを読み込む（同期的に待機）
   * slaveId: スレーブのID
   * startAddr: 開始アドレス
   * buffer: データを格納するバッファ
   * count: 読み込むレジスタ数
   * 戻り値: 成功時true、タイムアウト時false
   */
  bool readRegistersSync(uint8_t slaveId, uint16_t startAddr, uint16_t* buffer, int count) {
    if (mb.readHreg(slaveId, startAddr, buffer, count)) {
      unsigned long start = millis();
      while (mb.slave()) {
        if (millis() - start > MODBUS_TIMEOUT) {
          Serial.printf("[MASTER] READ TIMEOUT from Slave %d\n", slaveId);
          return false;
        }
        mb.task();
      }
      return true;
    }
    return false;
  }

  /**
   * スレーブに複数レジスタを書き込む（同期的に待機）
   * slaveId: スレーブのID
   * startAddr: 開始アドレス
   * buffer: 書き込むデータ
   * count: 書き込むレジスタ数
   * 戻り値: 成功時true、タイムアウト時false
   */
  bool writeRegistersSync(uint8_t slaveId, uint16_t startAddr, const uint16_t* buffer, int count) {
    if (mb.writeHreg(slaveId, startAddr, (uint16_t*)buffer, count)) {
      unsigned long start = millis();
      while (mb.slave()) {
        if (millis() - start > MODBUS_TIMEOUT) {
          Serial.printf("[MASTER] WRITE TIMEOUT to Slave %d\n", slaveId);
          return false;
        }
        mb.task();
      }
      return true;
    }
    return false;
  }

  /**
   * 単一レジスタを読み込む（便利メソッド）
   */
  bool readRegister(uint8_t slaveId, uint16_t addr, uint16_t& value) {
    return readRegistersSync(slaveId, addr, &value, 1);
  }

  /**
   * 単一レジスタに書き込む（便利メソッド）
   */
  bool writeRegister(uint8_t slaveId, uint16_t addr, uint16_t value) {
    return writeRegistersSync(slaveId, addr, &value, 1);
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

private:
  /**
   * ボーレート変更（内部用）
   */
  void changeBaud(int baudIdx) {
    long newBaud = BAUDRATES[baudIdx];
    Serial.printf("[MASTER] Switching to %ld baud...\n", newBaud);
    Serial.flush();

    serial->flush();
    serial->end();
    delay(50);
    serial->begin(newBaud, SERIAL_8N1, -1, -1);
    delay(50);

    // トランシーバーリセット
    if (dePin >= 0) {
      pinMode(dePin, OUTPUT);
      digitalWrite(dePin, LOW);
    }

    currentBaudIdx = baudIdx;
    Serial.printf("[MASTER] Now at %ld baud\n\n", newBaud);
  }
};

#endif  // MODBUS_MASTER_H
