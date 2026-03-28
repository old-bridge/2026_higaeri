#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <Arduino.h>
#include <ModbusRTU.h>
#include "ModbusConfig.h"

class ModbusMaster {
public:
  ModbusMaster(HardwareSerial& serial, uint8_t dePin);

  void begin();
  void task();
  bool readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t* buffer, uint16_t count);
  bool writeHoldingRegisters(uint8_t slaveId, uint16_t startAddress, const uint16_t* values, uint16_t count);

private:
  static bool handleReadResult(Modbus::ResultCode event, uint16_t transactionId, void* data);
  static bool handleWriteResult(Modbus::ResultCode event, uint16_t transactionId, void* data);
  bool waitForResponse();

  ModbusRTU modbus_;
  HardwareSerial* serial_;
  uint8_t dePin_;
  bool waitingForResponse_;
  Modbus::ResultCode lastResult_;

  static ModbusMaster* activeInstance_;
};

#endif
