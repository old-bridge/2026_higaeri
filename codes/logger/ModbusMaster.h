#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <Arduino.h>
#include <ModbusRTU.h>

#include "ModbusConfig.h"

class ModbusMaster {
public:
  ModbusMaster(HardwareSerial& serial, uint8_t dePin, uint8_t rxPin, uint8_t txPin);

  void begin();
  void task();
  bool isBusy();
  bool readHoldingRegistersAsync(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, uint16_t count, cbTransaction callback);
  bool writeHoldingRegistersAsync(uint8_t slaveId, uint16_t startAddress, const uint16_t* values, uint16_t count, cbTransaction callback);

private:
  ModbusRTU modbus_;
  HardwareSerial* serial_;
  uint8_t dePin_;
  uint8_t rxPin_;
  uint8_t txPin_;
};

#endif
