#include "ModbusMaster.h"

/**
 * ModbusMaster::begin()
 * 初期化 - ModbusRTUClient の startModbusClient() を使用
 */
void ModbusMaster::begin() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("\n=== MODBUS MASTER/CLIENT ===");

  pinMode(dePin, OUTPUT);
  digitalWrite(dePin, LOW);

  // 9600 bauで開始
  serial->begin(9600, SERIAL_8N1, -1, -1);
  
  // ModbusRTUClient で Modbus クライアント（マスター）として初期化
  // すべてのスレーブのマスターとして動作するため address=0 (マスター用)
  // 実装に応じて address を設定（通常マスターは 0 または固定値）
  mb.startModbusClient(0, 9600, *serial, false);

  Serial.println("[MASTER] READY @ 9600 baud");
}

/**
 * ModbusMaster::task()
 * Modbusタスク実行（非同期処理が必要な場合）
 * 注意: ModbusRTUClient のメソッドはブロッキング/同期的です
 */
void ModbusMaster::task() {
  // ModbusRTUClient は synchronous なので、task() では特に処理なし
  // 必要に応じてタイムアウト監視や状態管理を追加
}

/**
 * ModbusMaster::changeBaud()
 * マスター側のボーレート切り替え
 */
void ModbusMaster::changeBaud(int baudIdx) {
  if (baudIdx < 0 || baudIdx >= NUM_BAUDS) {
    Serial.printf("[MASTER] Invalid baud index: %d\n", baudIdx);
    return;
  }

  if (baudIdx == currentBaudIdx) {
    return;  // 同じボーレート指定は無視
  }

  long newBaud = BAUDRATES[baudIdx];
  Serial.printf("[MASTER] Switching to %ld baud...\n", newBaud);
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
  Serial.printf("[MASTER] Now at %ld baud\n\n", newBaud);
}

/**
 * ModbusMaster::requestBaudChange()
 * スレーブのボーレート切り替えをリクエスト
 * 
 * 注意: ModbusRTUClient は同期的に動作するため、
 * write → delay → read の流れで実装
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
  int ret = mb.WriteSingleRegister(requestSlaveId << 8 | 300, baudIdx, 200000);  
  // Modbus address format: (slaveId << 8) | registerAddress
  
  if (ret == 0) {
    Serial.printf("[MASTER] Baud request sent successfully\n");
    delay(500);  // スレーブ側の処理を待つ
    
    // マスター側もボーレート切り替え
    changeBaud(newBaudIdx);
  } else {
    Serial.printf("[MASTER] Baud request failed: %d\n", ret);
  }
}

/**
 * ModbusMaster::readRegistersSync()
 * スレーブから複数ホールディングレジスタを読み込む（同期的）
 * 
 * @param slaveId スレーブID
 * @param startAddr レジスタ開始アドレス
 * @param buffer 読み込みバッファ
 * @param count 読み込みレジスタ数
 * @return true if successful
 */
bool ModbusMaster::readRegistersSync(uint8_t slaveId, uint16_t startAddr, uint16_t* buffer, int count) {
  // ModbusRTUClient は device address を自動的に処理
  // address parameter: (slaveId << 8) | registerAddress
  int ret = mb.ReadHoldingRegisters(startAddr, count, buffer, 200000);
  
  if (ret == 0) {
    return true;
  } else {
    Serial.printf("[MASTER] Read failed from Slave %d: %d\n", slaveId, ret);
    return false;
  }
}

/**
 * ModbusMaster::writeRegistersSync()
 * スレーブに複数ホールディングレジスタを書き込む（同期的）
 * 
 * @param slaveId スレーブID
 * @param startAddr レジスタ開始アドレス
 * @param buffer 書き込みバッファ
 * @param count 書き込みレジスタ数
 * @return true if successful
 */
bool ModbusMaster::writeRegistersSync(uint8_t slaveId, uint16_t startAddr, const uint16_t* buffer, int count) {
  int ret = mb.WriteMultipleRegisters(startAddr, count, (uint16_t*)buffer, 200000);
  
  if (ret == 0) {
    return true;
  } else {
    Serial.printf("[MASTER] Write failed to Slave %d: %d\n", slaveId, ret);
    return false;
  }
}
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
 * Other helper methods can be added here as needed
 */
