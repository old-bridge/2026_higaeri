// logger.ino - MASTER (Modbus RTU Master)
// XIAO ESP32C3
//
// DE/RE wiring: both DE and /RE of MAX3485 are tied to DE_PIN (same GPIO).
//   GPIO=HIGH → TX mode (driver on, receiver off)
//   GPIO=LOW  → RX mode (driver off, receiver on)
//
// DE control is handled INSIDE the library (modified defaultSerialWriteFunction):
//   before serial->write()  : DE = HIGH
//   after  serial->flush()  : DE = LOW
// Just call setDEPin(DE_PIN) once and the library does the rest.

#include <HardwareSerial.h>
#include <ModbusRTU.h>

#define BAUDRATE          9600UL
#define DE_PIN            4       // GPIO4 = D2
#define SLAVE_ID_AIR_DATA 1
#define SLAVE_ID_DISPLAY  2
#define REGN_SENSOR       0       // Holding registers 0-3: sensor data
#define SENSOR_DATA_SIZE  4
#define LED_PIN D0

// UART0 on XIAO ESP32C3: TX=GPIO21(D6), RX=GPIO20(D7)
// Use -1,-1 to select UART0 default pins.
HardwareSerial MySerial0(0);

ModbusRTUClient mbAirData;  // Talks to air_data (Slave ID=1)
ModbusRTUClient mbDisplay;  // Talks to display_d1 (Slave ID=2)

// Read SENSOR_DATA_SIZE holding registers from a slave. Returns true on success.
uint16_t g_readBuf[SENSOR_DATA_SIZE] = {0};

bool readFromSlave(ModbusRTUClient& mb, uint16_t reg, uint16_t count, uint16_t* buf) {
  return mb.ReadHoldingRegisters(reg, count, buf, 200000UL, false) == 0;
}

// -------------------------------------------------------------------
unsigned long lastReadMs = 0;
uint32_t cycleCount = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[LOGGER MASTER] Starting...");

  // Initialize UART0 with Modbus RTU standard format (8E1)
  MySerial0.begin(BAUDRATE, SERIAL_8E1, -1, -1);

  // Initialize DE pin in receive mode
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);

  // Initialize Modbus clients.
  // initialize=false because we already called MySerial0.begin().
  mbAirData.startModbusClient(SLAVE_ID_AIR_DATA, BAUDRATE, MySerial0, false);
  mbDisplay.startModbusClient(SLAVE_ID_DISPLAY,  BAUDRATE, MySerial0, false);

  // Tell each client which GPIO to use for DE/RE control.
  // The library's defaultSerialWriteFunction will now handle:
  //   DE=HIGH before write, flush(), DE=LOW after transmission.
  mbAirData.setDEPin(DE_PIN);
  mbDisplay.setDEPin(DE_PIN);

  Serial.printf("[LOGGER MASTER] baud=%lu  de=GPIO%d  air_data=SlaveID%d  display=SlaveID%d\n",
    BAUDRATE, DE_PIN, SLAVE_ID_AIR_DATA, SLAVE_ID_DISPLAY);
}

void loop() {
  if (millis() - lastReadMs < 1000) return;
  lastReadMs = millis();
  cycleCount++;
  Serial.printf("\n[LOGGER MASTER] ===== cycle %lu =====\n", (unsigned long)cycleCount);

  // --- Read air_data slave (ID=1) ---
  if (readFromSlave(mbAirData, REGN_SENSOR, SENSOR_DATA_SIZE, g_readBuf)) {
    Serial.printf("[LOGGER MASTER] air_data  rot=%u  as1=%u  as2=%u  batt=%u\n",
      g_readBuf[0], g_readBuf[1], g_readBuf[2], g_readBuf[3]);
  } else {
    Serial.println("[LOGGER MASTER] air_data  READ FAILED");
  }

  delay(20);  // Inter-message gap (let bus settle)

  // --- Read display_d1 slave (ID=2) ---
  if (readFromSlave(mbDisplay, REGN_SENSOR, SENSOR_DATA_SIZE, g_readBuf)) {
    Serial.printf("[LOGGER MASTER] display   d0=%u  d1=%u  d2=%u  d3=%u\n",
      g_readBuf[0], g_readBuf[1], g_readBuf[2], g_readBuf[3]);
  } else {
    Serial.println("[LOGGER MASTER] display   READ FAILED");
  }
}