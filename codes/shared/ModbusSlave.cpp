#include "ModbusSlave.h"

ModbusSlaveBase* ModbusSlaveBase::activeInstance_ = nullptr;

ModbusSlaveBase::ModbusSlaveBase(HardwareSerial& serial, uint8_t slaveId, uint8_t dePin)
  : serial_(&serial), slaveId_(slaveId), dePin_(dePin) {}

void ModbusSlaveBase::begin() {
  pinMode(dePin_, OUTPUT);
  digitalWrite(dePin_, LOW);

  serial_->begin(kModbusBaudRate, SERIAL_8N1, kModbusRxPin, kModbusTxPin);
  modbus_.begin(serial_, dePin_);
  modbus_.slave(slaveId_);

  activeInstance_ = this;
  setupRegisters();
  setupCallbacks();
}

void ModbusSlaveBase::task() {
  modbus_.task();
}

void ModbusSlaveBase::setupCallbacks() {
}

uint16_t ModbusSlaveBase::onReadRegister(TRegister*, uint16_t currentValue) {
  return currentValue;
}

uint16_t ModbusSlaveBase::onWriteRegister(TRegister*, uint16_t newValue) {
  return newValue;
}

void ModbusSlaveBase::addHoldingRegisters(uint16_t startAddress, uint16_t count, uint16_t initialValue) {
  modbus_.addHreg(startAddress, initialValue, count);
}

void ModbusSlaveBase::registerReadHandler(uint16_t startAddress, uint16_t count) {
  modbus_.onGetHreg(startAddress, handleReadThunk, count);
}

void ModbusSlaveBase::registerWriteHandler(uint16_t startAddress, uint16_t count) {
  modbus_.onSetHreg(startAddress, handleWriteThunk, count);
}

void ModbusSlaveBase::setHoldingValue(uint16_t address, uint16_t value) {
  modbus_.Hreg(address, value);
}

uint16_t ModbusSlaveBase::getHoldingValue(uint16_t address) const {
  return const_cast<ModbusRTU&>(modbus_).Hreg(address);
}

uint16_t ModbusSlaveBase::handleReadThunk(TRegister* reg, uint16_t currentValue) {
  if (activeInstance_ == nullptr) {
    return currentValue;
  }
  return activeInstance_->onReadRegister(reg, currentValue);
}

uint16_t ModbusSlaveBase::handleWriteThunk(TRegister* reg, uint16_t newValue) {
  if (activeInstance_ == nullptr) {
    return newValue;
  }
  return activeInstance_->onWriteRegister(reg, newValue);
}
