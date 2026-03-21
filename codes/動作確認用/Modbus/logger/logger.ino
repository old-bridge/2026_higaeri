// NOTE: Do NOT use USE_CUSTOM_READ_WRITE_FUNCTIONS due to library bug
// (ModbusRTUClient has no startModbusClient() method in that mode)
// Instead, we use default API with manual DE_PIN control after each operation

#include <ModbusRTU.h>

#define BAUDRATE 9600UL
#define DE_PIN 4          // GPIO 4 (D2)
#define RX_PIN -1         // Default RX
#define TX_PIN -1         // Default TX
#define SLAVE_ID 1
#define REGN_SENSOR 0     // Sensor data registers (0-3: rot, as1, as2, batt)
#define REGN_CONTROL 10   // Control register

const int datasize = 50;

ModbusRTUServer mb;
// Use Serial0 (HardwareSerial) instead of Serial (HWCDC)
// Serial0 = UART 0 (hardware UART on XIAO ESP32C3)

uint16_t holdingRegs[HOLDING_REGISTER_NUM] = {0};

unsigned long lastLogTime = 0;
unsigned long lastStatusTime = 0;
uint32_t logCounter = 0;

// DE_PIN control
void setDETransmit() {
  digitalWrite(DE_PIN, HIGH);
  delayMicroseconds(100);
}

void setDEReceive() {
  digitalWrite(DE_PIN, LOW);
  delayMicroseconds(100);
}

// Helper: Flush serial buffer and switch to receive mode
void finishTransmission() {
  Serial0.flush();
  delayMicroseconds(500);
  setDEReceive();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[LOGGER SLAVE] Starting...");

  // Initialize serial with EVEN PARITY (Modbus RTU standard)
  // Use Serial0, not Serial which is HWCDC on ESP32
  Serial0.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN);
  pinMode(DE_PIN, OUTPUT);
  setDEReceive();

  // Initialize Modbus server (slave)
  mb.startModbusServer(SLAVE_ID, BAUDRATE, Serial0, false);

  Serial.printf("[LOGGER SLAVE] id=%u baud=%lu de=%u (SERIAL_8E1)\n", SLAVE_ID, BAUDRATE, DE_PIN);
}

void loop() {
  // Process Modbus communication
  bool newData = mb.communicationLoop();

  // Log sensor data from registers (updated by master writes)
  if (millis() - lastLogTime >= 1000) {
    lastLogTime = millis();
    logCounter++;

    uint16_t rot = 0, as1 = 0, as2 = 0, batt = 0;
    mb.copyFromHoldingRegisters(&rot, 1, REGN_SENSOR + 0);
    mb.copyFromHoldingRegisters(&as1, 1, REGN_SENSOR + 1);
    mb.copyFromHoldingRegisters(&as2, 1, REGN_SENSOR + 2);
    mb.copyFromHoldingRegisters(&batt, 1, REGN_SENSOR + 3);

    Serial.printf("[LOGGER SLAVE] log_count=%lu rot=%u as1=%u as2=%u batt=%u\n",
      (unsigned long)logCounter, rot, as1, as2, batt);

    // TODO: Write to SD card
  }

  // Print status periodically
  if (millis() - lastStatusTime >= 5000) {
    lastStatusTime = millis();
    Serial.printf("[LOGGER SLAVE] status logs=%lu\n", (unsigned long)logCounter);
  }
}