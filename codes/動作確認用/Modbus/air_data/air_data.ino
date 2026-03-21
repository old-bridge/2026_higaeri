// NOTE: Do NOT use USE_CUSTOM_READ_WRITE_FUNCTIONS due to library bug
// (ModbusRTUClient has no startModbusClient() method in that mode)
// Instead, we use default API with manual DE_PIN control after each operation

#include <ModbusRTU.h>

// Configuration
#define BAUDRATE 9600UL
#define DE_PIN 0           // GPIO 0 (D10)
#define RX_PIN -1          // Default RX
#define TX_PIN -1          // Default TX
#define SLAVE_ID_LOGGER 1
#define SLAVE_ID_DISPLAY 2
#define REGN_SENSOR 0      // Sensor data registers (0-3: rot, as1, as2, batt)

// Data sizes
const int SENSOR_DATA_SIZE = 4;  // rot, AS5600①, AS5600②, battery

// Modbus objects (separate client for each slave)
ModbusRTUClient mbLogger;
ModbusRTUClient mbDisplay;
// Use Serial0 (HardwareSerial) instead of Serial (HWCDC)
// Serial0 = UART 0 (hardware UART on XIAO ESP32C3)

// Sensor data buffers
uint16_t sensorData[SENSOR_DATA_SIZE] = {0};
uint16_t readBuffer[SENSOR_DATA_SIZE] = {0};

unsigned long lastWriteTime = 0;
unsigned long lastReadTime = 0;
uint32_t cycleCounter = 0;

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
  delayMicroseconds(500);  // Let all bits transmit before switching direction
  setDEReceive();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[AIR_DATA MASTER] Starting...");

  // Initialize serial (use Serial0, not Serial which is HWCDC on ESP32)
  // IMPORTANT: Modbus RTU standard requires EVEN PARITY (8E1)
  Serial0.begin(BAUDRATE, SERIAL_8E1, RX_PIN, TX_PIN);
  pinMode(DE_PIN, OUTPUT);
  setDEReceive();

  // Initialize Modbus master client for Logger (Slave ID = 1)
  mbLogger.startModbusClient(SLAVE_ID_LOGGER, BAUDRATE, Serial0, false);

  // Initialize Modbus master client for Display (Slave ID = 2)
  mbDisplay.startModbusClient(SLAVE_ID_DISPLAY, BAUDRATE, Serial0, false);

  Serial.printf("[AIR_DATA MASTER] baud=%lu de=%u initialized\n", BAUDRATE, DE_PIN);
}

void writeSensorDataToSlaves() {
  sensorData[0] = (uint16_t)(millis() / 100) % 360;
  sensorData[1] = (uint16_t)(millis() / 50) % 4096;
  sensorData[2] = (uint16_t)(millis() / 75) % 4096;
  sensorData[3] = 3300 + ((millis() / 10) % 200);

  setDETransmit();
  int resultLogger = mbLogger.WriteMultipleRegisters(REGN_SENSOR, SENSOR_DATA_SIZE, sensorData, 500000, false);
  finishTransmission();
  
  if (resultLogger == 0) {
    Serial.printf("[AIR_DATA MASTER] wrote to logger: rot=%u as1=%u as2=%u batt=%u\n",
      sensorData[0], sensorData[1], sensorData[2], sensorData[3]);
  } else {
    Serial.printf("[AIR_DATA MASTER] logger write failed: %d\n", resultLogger);
  }

  delay(100);

  setDETransmit();
  int resultDisplay = mbDisplay.WriteMultipleRegisters(REGN_SENSOR, SENSOR_DATA_SIZE, sensorData, 500000, false);
  finishTransmission();
  
  if (resultDisplay == 0) {
    Serial.printf("[AIR_DATA MASTER] wrote to display: rot=%u as1=%u as2=%u batt=%u\n",
      sensorData[0], sensorData[1], sensorData[2], sensorData[3]);
  } else {
    Serial.printf("[AIR_DATA MASTER] display write failed: %d\n", resultDisplay);
  }
}

void readFromSlaves() {
  setDETransmit();
  int resultLogger = mbLogger.ReadHoldingRegisters(REGN_SENSOR, SENSOR_DATA_SIZE, readBuffer, 500000, false);
  finishTransmission();
  
  if (resultLogger == 0) {
    Serial.printf("[AIR_DATA MASTER] read logger: rot=%u as1=%u as2=%u batt=%u\n",
      readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);
  } else {
    Serial.printf("[AIR_DATA MASTER] logger read failed: %d\n", resultLogger);
  }

  delay(100);

  setDETransmit();
  int resultDisplay = mbDisplay.ReadHoldingRegisters(REGN_SENSOR, SENSOR_DATA_SIZE, readBuffer, 500000, false);
  finishTransmission();
  
  if (resultDisplay == 0) {
    Serial.printf("[AIR_DATA MASTER] read display: rot=%u as1=%u as2=%u batt=%u\n",
      readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3]);
  } else {
    Serial.printf("[AIR_DATA MASTER] display read failed: %d\n", resultDisplay);
  }
}

void loop() {
  // Write sensor data to slaves
  if (millis() - lastWriteTime >= 500) {
    lastWriteTime = millis();
    writeSensorDataToSlaves();
  }

  // Read back from slaves for verification
  if (millis() - lastReadTime >= 1000) {
    lastReadTime = millis();
    cycleCounter++;
    readFromSlaves();
    Serial.printf("[AIR_DATA MASTER] cycle=%lu\n", (unsigned long)cycleCounter);
  }
}