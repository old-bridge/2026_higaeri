#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <Arduino.h>
#include <ModbusRTU.h>
#include "ModbusConfig.h"

class ModbusSlaveBase {
public:
  ModbusSlaveBase(HardwareSerial& serial, uint8_t slaveId, uint8_t dePin);
  virtual ~ModbusSlaveBase() = default;

  void begin();
  void task();

protected:
  virtual void setupRegisters() = 0;
  virtual void setupCallbacks();
  virtual uint16_t onReadRegister(TRegister* reg, uint16_t currentValue);
  virtual uint16_t onWriteRegister(TRegister* reg, uint16_t newValue);

  void addHoldingRegisters(uint16_t startAddress, uint16_t count, uint16_t initialValue = 0);
  void registerReadHandler(uint16_t startAddress, uint16_t count = 1);
  void registerWriteHandler(uint16_t startAddress, uint16_t count = 1);
  void setHoldingValue(uint16_t address, uint16_t value);
  uint16_t getHoldingValue(uint16_t address) const;

  ModbusRTU modbus_;

private:
  static uint16_t handleReadThunk(TRegister* reg, uint16_t currentValue);
  static uint16_t handleWriteThunk(TRegister* reg, uint16_t newValue);

  HardwareSerial* serial_;
  uint8_t slaveId_;
  uint8_t dePin_;

  static ModbusSlaveBase* activeInstance_;
};

#endif
