#include "ModbusSlave.h"

/**
 * ModbusSlaveBase::begin()
 * 初期化
 */
void ModbusSlaveBase::begin() {
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
 * ModbusSlaveBase::task()
 * Modbusタスク実行
 */
void ModbusSlaveBase::task() {
  mb.task();
}

/**
 * ModbusSlaveBase::changeBaud()
 * ボーレート切り替え
 */
void ModbusSlaveBase::changeBaud(int baudIdx) {
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
  serial->begin(newBaud, SERIAL_8N1, 5, 4);  // RX: D7(GPIO5), TX: D6(GPIO4)
  delay(200);

  // トランシーバーリセット
  digitalWrite(dePin, HIGH);
  delay(5);
  digitalWrite(dePin, LOW);

  currentBaudIdx = baudIdx;
  Serial.printf("[SLAVE %d] Now at %ld baud\n\n", slaveId, newBaud);
}

/**
 * ModbusSlaveBase::getCurrentBaud()
 * 現在のボーレート取得
 */
long ModbusSlaveBase::getCurrentBaud() const {
  return BAUDRATES[currentBaudIdx];
}

/**
 * ModbusSlaveBase::getCurrentBaudIdx()
 * 現在のボーレートインデックス取得
 */
int ModbusSlaveBase::getCurrentBaudIdx() const {
  return currentBaudIdx;
}

/**
 * ModbusSlaveBase::setRegisterValue()
 * レジスタに値を書き込む
 */
void ModbusSlaveBase::setRegisterValue(uint16_t address, uint16_t value) {
  mb.Hreg(address, value);
}

/**
 * ModbusSlaveBase::getRegisterValue()
 * レジスタの値を読み込む
 */
uint16_t ModbusSlaveBase::getRegisterValue(uint16_t address) const {
  return mb.Hreg(address);
}
