#include "ModbusMaster.h"

ModbusMaster* ModbusMaster::activeInstance_ = nullptr;

ModbusMaster::ModbusMaster(HardwareSerial& serial, uint8_t dePin, uint8_t rxPin, uint8_t txPin)
  : serial_(&serial),
    dePin_(dePin),
    rxPin_(rxPin),
    txPin_(txPin),
    waitingForResponse_(false),
    lastResult_(Modbus::EX_GENERAL_FAILURE) {}

void ModbusMaster::begin() {
  pinMode(dePin_, OUTPUT);
  digitalWrite(dePin_, LOW);

  serial_->begin(kModbusBaudRate, SERIAL_8N1, rxPin_, txPin_);
  modbus_.begin(serial_, dePin_);
  modbus_.master();
  activeInstance_ = this;
}

void ModbusMaster::task() {
  modbus_.task();
}

bool ModbusMaster::isBusy() {
  return modbus_.slave() != 0;
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

bool ModbusMaster::readHoldingRegistersAsync(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, uint16_t count, cbTransaction callback) {
  return modbus_.readHreg(slaveId, startAddress, buffer, count, callback);
}

bool ModbusMaster::writeHoldingRegistersAsync(uint8_t slaveId, uint16_t startAddress, const uint16_t* values, uint16_t count, cbTransaction callback) {
  return modbus_.writeHreg(slaveId, startAddress, const_cast<uint16_t*>(values), count, callback);
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
