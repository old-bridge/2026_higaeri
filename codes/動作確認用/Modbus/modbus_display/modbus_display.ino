// display_d1.ino - SLAVE (Modbus RTU Slave, ID = 2)
// XIAO ESP32C3
//
// DE/RE wiring: both DE and /RE of MAX3485 are tied to DE_PIN (same GPIO).
//   GPIO=HIGH → TX mode (driver on, receiver off)
//   GPIO=LOW  → RX mode (driver off, receiver on)
//
// DE control is handled INSIDE the library (modified defaultSerialWriteFunction):
//   before serial->write()  : DE = HIGH
//   after  serial->flush()  : DE = LOW
// Just call setDEPin(DE_PIN) once. No callbacks needed.

#include <HardwareSerial.h>
#include <ModbusRTU.h>

#define BAUDRATE     9600UL
#define DE_PIN       5        // GPIO5 = D3
#define SLAVE_ID     2
#define REGN_SENSOR  0        // Holding registers 0-3: data

// UART0 on XIAO ESP32C3: TX=GPIO21(D6), RX=GPIO20(D7)
HardwareSerial MySerial0(0);
ModbusRTUServer mb;

unsigned long lastPrintMs = 0;
uint32_t loopCount = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[DISPLAY_D1 SLAVE] Starting...");

  // Initialize UART0 with Modbus RTU standard format (8E1)
  MySerial0.begin(BAUDRATE, SERIAL_8E1, -1, -1);

  // Initialize DE pin in receive mode (default)
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);

  // Initialize Modbus server (slave).
  // initialize=false because we already called MySerial0.begin().
  mb.startModbusServer(SLAVE_ID, BAUDRATE, MySerial0, false);

  // Tell the library which GPIO to use for DE/RE.
  mb.setDEPin(DE_PIN);

  // Pre-fill holding registers with initial values
  for (uint16_t i = 0; i < 4; i++) mb.setHoldingValue(REGN_SENSOR + i, 0);

  Serial.printf("[DISPLAY_D1 SLAVE] id=%u  baud=%lu  de=GPIO%d\n", SLAVE_ID, BAUDRATE, DE_PIN);
}

void loop() {
  // Update simulated data in holding registers
  mb.setHoldingValue(REGN_SENSOR + 0, (uint16_t)(millis() / 200) % 1000);
  mb.setHoldingValue(REGN_SENSOR + 1, (uint16_t)(millis() / 150) % 1000);
  mb.setHoldingValue(REGN_SENSOR + 2, (uint16_t)(millis() / 300) % 1000);
  mb.setHoldingValue(REGN_SENSOR + 3, (uint16_t)(millis() / 250) % 1000);

  // Process incoming Modbus request (non-blocking when idle).
  // The library now handles DE automatically inside defaultSerialWriteFunction.
  mb.communicationLoop();

  // Print current register values periodically
  if (millis() - lastPrintMs >= 2000) {
    lastPrintMs = millis();
    Serial.printf("[DISPLAY_D1 SLAVE] regs: %u %u %u %u\n",
      mb.getHoldingValue(REGN_SENSOR + 0),
      mb.getHoldingValue(REGN_SENSOR + 1),
      mb.getHoldingValue(REGN_SENSOR + 2),
      mb.getHoldingValue(REGN_SENSOR + 3));
  }

  if (++loopCount % 50000 == 0) {
    Serial.printf("[DISPLAY_D1 SLAVE] alive  loop=%lu\n", (unsigned long)loopCount);
  }
}