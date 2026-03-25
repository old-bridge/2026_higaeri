// air_data.ino - SLAVE (Modbus RTU Slave, ID = 1)
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
#define DE_PIN       10      // GPIO10 = D10  (was incorrectly 0)
#define SLAVE_ID     1
#define REGN_SENSOR  0       // Holding registers 0-3: rot, as1, as2, batt
#define LED_PIN D0


// UART0 on XIAO ESP32C3: TX=GPIO21(D6), RX=GPIO20(D7)
HardwareSerial MySerial0(0);
ModbusRTUServer mb;

uint32_t loopCount = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED ON to indicate setup start
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[AIR_DATA SLAVE] Starting...");

  // Initialize UART0 with Modbus RTU standard format (8E1)
  MySerial0.begin(BAUDRATE, SERIAL_8E1, -1, -1);

  // Initialize DE pin in receive mode (default)
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);

  // Initialize Modbus server (slave).
  // initialize=false because we already called MySerial0.begin().
  mb.startModbusServer(SLAVE_ID, BAUDRATE, MySerial0, false);

  // Tell the library which GPIO to use for DE/RE.
  // defaultSerialWriteFunction will raise DE before write, flush, then drop DE.
  mb.setDEPin(DE_PIN);

  // Pre-fill holding registers with initial values
  for (uint16_t i = 0; i < 4; i++) mb.setHoldingValue(REGN_SENSOR + i, 0);

  Serial.printf("[AIR_DATA SLAVE] id=%u  baud=%lu  de=GPIO%d\n", SLAVE_ID, BAUDRATE, DE_PIN);
}

void loop() {
  // Update simulated sensor data in holding registers
  mb.setHoldingValue(REGN_SENSOR + 0, (uint16_t)(millis() / 100) % 360);
  mb.setHoldingValue(REGN_SENSOR + 1, (uint16_t)(millis() / 50)  % 4096);
  mb.setHoldingValue(REGN_SENSOR + 2, (uint16_t)(millis() / 75)  % 4096);
  mb.setHoldingValue(REGN_SENSOR + 3, 3300 + (uint16_t)((millis() / 10) % 200));

  // ---- DIAGNOSTIC BLOCK ----
  // Check if request bytes are arriving in UART RX buffer
  int rxAvail = MySerial0.available();
  if (rxAvail > 0) {
    Serial.printf("[DIAG] RX bytes in buffer: %d  (expected 8 for Modbus request)\n", rxAvail);
  }
  // --------------------------

  // DE_PIN sanity check: confirm the pin is actually responding.
  // Every ~1s, briefly toggle DE_PIN and print the readback value.
  static unsigned long lastDeCheckMs = 0;
  if (millis() - lastDeCheckMs >= 1000) {
    lastDeCheckMs = millis();
    digitalWrite(DE_PIN, HIGH);
    delayMicroseconds(10);
    int deHigh = digitalRead(DE_PIN);
    digitalWrite(DE_PIN, LOW);
    delayMicroseconds(10);
    int deLow = digitalRead(DE_PIN);
    Serial.printf("[DIAG] DE_PIN=GPIO%d  readback: HIGH=%d LOW=%d  (expected 1 and 0)\n",
      DE_PIN, deHigh, deLow);
  }

  mb.communicationLoop();

  if (++loopCount % 50000 == 0) {
    Serial.printf("[AIR_DATA SLAVE] alive  loop=%lu  reg0=%u\n",
      (unsigned long)loopCount, mb.getHoldingValue(REGN_SENSOR + 0));
  }
}