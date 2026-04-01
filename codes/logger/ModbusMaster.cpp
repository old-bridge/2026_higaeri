#include "ModbusMaster.h"

ModbusMaster::ModbusMaster(HardwareSerial& serial, uint8_t dePin, uint8_t rxPin, uint8_t txPin)
  : serial_(&serial),
    dePin_(dePin),
    rxPin_(rxPin),
    txPin_(txPin) {}

void ModbusMaster::begin() {
  pinMode(dePin_, OUTPUT);
  digitalWrite(dePin_, LOW);

  serial_->begin(kModbusBaudRate, SERIAL_8N1, rxPin_, txPin_);
  modbus_.begin(serial_, dePin_);
  modbus_.master();
}

void ModbusMaster::task() {
  modbus_.task();
}

bool ModbusMaster::isBusy() {
  return modbus_.slave() != 0;
}

bool ModbusMaster::readHoldingRegistersAsync(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, uint16_t count, cbTransaction callback) {
  return modbus_.readHreg(slaveId, startAddress, buffer, count, callback);
}

bool ModbusMaster::writeHoldingRegistersAsync(uint8_t slaveId, uint16_t startAddress, const uint16_t* values, uint16_t count, cbTransaction callback) {
  return modbus_.writeHreg(slaveId, startAddress, const_cast<uint16_t*>(values), count, callback);
}
