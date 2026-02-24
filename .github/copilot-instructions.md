# AI Coding Agent Instructions - Modbus RTU Flight Control System

## 📋 System Architecture

This is a **Modbus RTU master-slave distributed system** for flight control with four Seeduino XIAO ESP32C3 microcontrollers:

- **Logger MCU (Master)**: Aggregates data from slaves, logs to MicroSD, manages baud rate synchronization
- **Air Data MCU (Slave ID: 1)**: Provides sensor data via Modbus (rotational encoder, AS5600 angles, battery voltage)
- **Display D-1 MCU (Slave ID: 2)**: TFT display controller, barometric altitude sensor, I2C master to D-2
- **Display D-2 MCU (I2C Slave to D-1)**: Potentiometers①②, battery voltage, ultrasonic altitude sensor

**Communication Pattern**:
- All Modbus slaves respond to logger's RTU queries (9600-5M baud variable)
- D-2 communicates with D-1 via I2C (400kHz) to share sensor data
- D-1 aggregates D-2's data and presents it to logger via Modbus

## 🔑 Critical Design Patterns

### 1. Modbus Register Model
- **Read Registers** (address 0-X): Slave → Master sensor data
  - Air Data: rotation count, AS5600①②angles, battery voltage
  - Display D-1: barometric altitude (D-2 data aggregated internally)
- **Write Registers** (address 10): Master → Slave control commands (LED, etc.)
- **Special Registers** (address 300): Baud rate switching index (0=9600, 1=38400, 2=115200, 3=921600, 4=5000000 bps)

**Modbus Slave IDs**:
- ID 1: Air Data MCU
- ID 2: Display D-1 MCU (TFT controller, serves as I2C master to D-2)
- ID 3: Reserved for future use

**Note**: Display D-2 is NOT a Modbus slave; it communicates only via I2C to D-1

**Key File**: [shared/ModbusConfig.h](shared/ModbusConfig.h) - All register addresses defined as constants (e.g., `AIR_REG_READ`, `DISP_REG_WRITE`)

### 2. Base Class Pattern
All slave MCUs inherit from `ModbusSlaveBase` ([shared/ModbusSlave.h](shared/ModbusSlave.h)):
- Subclasses **must override** `setupRegisters()` and `setupCallbacks()`
- Static callback functions for register read/write events
- Built-in baud rate switching via `changeBaud(baudIdx)`

Master uses `ModbusMaster` class ([shared/ModbusMaster.h](shared/ModbusMaster.h)) with:
- `readRegistersSync()` for synchronous Modbus reads
- `requestBaudChange()` for coordinating baud switches across all slaves

### 3. Serial & RS485 Control
- Each MCU has unique **DE_PIN** (driver-enable for RS485):
  - Logger: D2, Air Data: D10, Display: D3
- `#define DE_PIN` **must be placed before** including `ModbusMaster.h` or `ModbusSlave.h`
- RS485 uses `HardwareSerial(0)` - never use USB serial for Modbus communication

## 📂 File Structure & Responsibilities

```
shared/
  ModbusConfig.h        ← Central register/baudrate/ID constants
  ModbusSlave.h         ← Base class for air_data & display slaves
  ModbusMaster.h        ← Master class for logger
  TFTDisplay.h          ← Base class for display GUI

logger/logger.ino       ← Master: reads from Air Data and Display D-1 every 1000ms
air_data/air_data.ino   ← Slave #1: sensor provider
display/
  display_d1.ino        ← Slave #2: TFT controller + I2C master to D-2
  display_d2.ino        ← I2C Slave: sensor provider (pot①②, battery, ultrasonic)
  User_Setup.h          ← TFT_eSPI configuration file
```

## ⚙️ Common Development Tasks

### Adding a New Sensor to Air Data Slave
1. Define register offset in [shared/ModbusConfig.h](shared/ModbusConfig.h) (e.g., `#define AIR_REG_TEMPERATURE 4`)
2. Expand `AIR_REG_READ_SIZE` constant
3. Update `setupRegisters()` in `AirDataSlave` class to add register
4. Implement callback or update sensor reading loop
5. **Sync logger**: Add read buffer and query in [logger/logger.ino](logger/logger.ino#L28)

### Adding a New Sensor to Display D-2
1. Add ADC/UART pin definition in [display/display_d2.ino](display/display_d2.ino)
2. Create `readNewSensor()` function to read hardware value
3. Update `SensorData` struct to include new field
4. Call `readNewSensor()` inside `updateSensors()` loop
5. Update `onI2CRequest()` to include new data in I2C buffer
6. **Sync D-1**: Display MCU automatically receives new data via I2C
7. **Sync logger**: Display D-1 now has access to new sensor via I2C (e.g., `displaySlave.getNewSensorValue()`)

### I2C Communication (D-1 ↔ D-2)
- **I2C Address**: D-2 = `0x30` (master: D-1)
- **Clock Speed**: 400kHz
- **Data Format**: 8 bytes = 4 uint16_t values (little-endian)
  - Bytes [0-1]: Potentiometer①
  - Bytes [2-3]: Potentiometer②
  - Bytes [4-5]: Battery Voltage
  - Bytes [6-7]: Ultrasonic Altitude
- **Update Frequency**: On-demand when D-1 calls `Wire.requestFrom(0x30, 8)`

## ⚠️ Project-Specific Gotchas

1. **DE_PIN order matters**: `#define DE_PIN` must come **before** `#include "../shared/Modbus*.h"`
2. **Static callbacks**: Register callbacks are static functions - cannot access instance variables directly
3. **Synchronous vs Async**: Logger uses `readRegistersSync()` but framework supports async - mixing requires callback management
4. **Baud array synchronization**: All MCUs using Modbus (Logger, Air Data, D-1) must share same `BAUDRATES[]` - changes require recompiling firmware
5. **I2C vs RS485**: Modbus communication on RS485; I2C used only for D-1↔D-2 local communication
6. **D-2 No Modbus**: Display D-2 is NOT a Modbus slave - only communicates via I2C to D-1
7. **I2C Buffer Format**: D-1 must decode little-endian uint16_t values from D-2 in `readSensorDataFromD2()`

## 📌 Key Reference Points

- **Register mapping table**: [README.md](README.md#-modbusレジスタマップ)
- **Callback signature**: `uint16_t cbName(TRegister* reg, uint16_t value)` - used in `onGetHreg()` / `onSetHreg()`
- **Read timing**: Display MCU TFT updates tied to Modbus read cadence (1Hz) for consistency
- **Baud switching protocol**: Logger sends write → waits 500ms → switches own baud (slaves detect change in register callback)
