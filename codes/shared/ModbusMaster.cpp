#include "ModbusMaster.h"

ModbusMaster* ModbusMaster::activeInstance_ = nullptr;

ModbusMaster::ModbusMaster(HardwareSerial& serial, uint8_t dePin)
  : serial_(&serial),
    dePin_(dePin),
    waitingForResponse_(false),
    lastResult_(Modbus::EX_GENERAL_FAILURE) {}

void ModbusMaster::begin() {
  pinMode(dePin_, OUTPUT);
  digitalWrite(dePin_, LOW);

  serial_->begin(kModbusBaudRate, SERIAL_8N1, kModbusRxPin, kModbusTxPin);
  modbus_.begin(serial_, dePin_);
  modbus_.master();
  activeInstance_ = this;
}

void ModbusMaster::task() {
  modbus_.task();
}

bool ModbusMaster::readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, uint16_t count) {
  waitingForResponse_ = true;
  lastResult_ = Modbus::EX_GENERAL_FAILURE;
  if (!modbus_.readHreg(slaveId, startAddress, buffer, count, handleReadResult)) {
    waitingForResponse_ = false;
    return false;
  }
  return waitForResponse();
}

bool ModbusMaster::writeHoldingRegisters(uint8_t slaveId, uint16_t startAddress, const uint16_t* values, uint16_t count) {
  waitingForResponse_ = true;
  lastResult_ = Modbus::EX_GENERAL_FAILURE;
  if (!modbus_.writeHreg(slaveId, startAddress, const_cast<uint16_t*>(values), count, handleWriteResult)) {
    waitingForResponse_ = false;
    return false;
  }
  return waitForResponse();
}

bool ModbusMaster::handleReadResult(Modbus::ResultCode event, uint16_t, void*) {
  if (activeInstance_ == nullptr) {
    return true;
  }
  activeInstance_->lastResult_ = event;
  activeInstance_->waitingForResponse_ = false;
  return true;
}

bool ModbusMaster::handleWriteResult(Modbus::ResultCode event, uint16_t, void*) {
  if (activeInstance_ == nullptr) {
    return true;
  }
  activeInstance_->lastResult_ = event;
  activeInstance_->waitingForResponse_ = false;
  return true;
}

bool ModbusMaster::waitForResponse() {
  const unsigned long startedAt = millis();
  while (waitingForResponse_) {
    task();
    yield();
    if (millis() - startedAt > kModbusTimeoutMs) {
      waitingForResponse_ = false;
      lastResult_ = Modbus::EX_TIMEOUT;
      return false;
    }
  }
  return lastResult_ == Modbus::EX_SUCCESS;
}
