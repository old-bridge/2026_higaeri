#include "ModbusMaster.h"

/**
 * ModbusMaster::begin()
 * 初期化
 */
void ModbusMaster::begin() {
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
 * ModbusMaster::task()
 * Modbusタスク実行
 */
void ModbusMaster::task() {
  mb.task();
}

/**
 * ModbusMaster::requestBaudChange()
 * ボーレート切り替えをリクエスト
 */
void ModbusMaster::requestBaudChange(uint8_t requestSlaveId, int newBaudIdx) {
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
  mb.writeHreg(requestSlaveId, 300, &baudIdx, 1);  // 300はボーレート制御レジスタ

  delay(500);  // スレーブ側の処理を待つ

  // マスター側もボーレート切り替え
  changeBaud(newBaudIdx);
}

/**
 * ModbusMaster::readRegistersSync()
 * スレーブから複数レジスタを読み込む（同期的に待機）
 */
bool ModbusMaster::readRegistersSync(uint8_t slaveId, uint16_t startAddr, uint16_t* buffer, int count) {
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
 * ModbusMaster::writeRegistersSync()
 * スレーブに複数レジスタを書き込む（同期的に待機）
 */
bool ModbusMaster::writeRegistersSync(uint8_t slaveId, uint16_t startAddr, const uint16_t* buffer, int count) {
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
 * ModbusMaster::readRegister()
 * 単一レジスタを読み込む（便利メソッド）
 */
bool ModbusMaster::readRegister(uint8_t slaveId, uint16_t addr, uint16_t& value) {
  return readRegistersSync(slaveId, addr, &value, 1);
}

/**
 * ModbusMaster::writeRegister()
 * 単一レジスタに書き込む（便利メソッド）
 */
bool ModbusMaster::writeRegister(uint8_t slaveId, uint16_t addr, uint16_t value) {
  return writeRegistersSync(slaveId, addr, &value, 1);
}

/**
 * ModbusMaster::getCurrentBaud()
 * 現在のボーレート取得
 */
long ModbusMaster::getCurrentBaud() const {
  return BAUDRATES[currentBaudIdx];
}

/**
 * ModbusMaster::getCurrentBaudIdx()
 * 現在のボーレートインデックス取得
 */
int ModbusMaster::getCurrentBaudIdx() const {
  return currentBaudIdx;
}

/**
 * ModbusMaster::changeBaud()
 * ボーレート変更（内部用）
 */
void ModbusMaster::changeBaud(int baudIdx) {
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
