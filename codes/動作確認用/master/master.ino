#include <ModbusRTU.h>

const int datasize = 50;
#define SLAVE_ID 1
#define REGN_READ 10  // Register for reading (value 100)
#define REGN_WRITE 100 // Register for writing (value 77 from master)
#define BAUDRATE 5000000



ModbusRTU mb;
HardwareSerial MySerial0(0);
uint16_t datarec[datasize]; // Array to store 1 holding register value
uint16_t datawrite[datasize];

unsigned long writeHregTime = 0;
unsigned long readHregTime = 0;

// Callback for read operation
bool cbRead(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.printf_P("Read request result: 0x%02X, Mem: %d, Trans ID: %d\n", event, ESP.getFreeHeap(), transactionId);
  if (event == Modbus::EX_SUCCESS) {
    Serial.print("3:callback readHreg:");
    Serial.print(datarec[0]); // Should print 100
    Serial.print(":");
    Serial.print(micros()-readHregTime);
    Serial.println("us.");

  } else {
    Serial.print("Read request failed with error: 0x");
    Serial.println(event, HEX);
  }
  return true;
}

// Callback for write operation
bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.printf_P(".  Write request result: 0x%02X, Mem: %d, Trans ID: %d\n", event, ESP.getFreeHeap(), transactionId);
  if (event == Modbus::EX_SUCCESS) {
    Serial.printf_P(".  Successfully wrote %d values to slave",datasize);
    Serial.print(micros()-writeHregTime);
    Serial.println("us to write callback.");
  } else {
    Serial.print("Write request failed with error: 0x");
    Serial.println(event, HEX);
  }
  return true;
}

void setup() {
  Serial.begin(BAUDRATE);
  MySerial0.begin(BAUDRATE, SERIAL_8N1, -1, -1); // Default RX/TX pins
  mb.begin(&MySerial0,D5);
  mb.master();
}

unsigned long lastReadTime = 0;  // Timer for read operation
unsigned long lastWriteTime = 0; // Timer for write operation
const unsigned long readInterval = 3300;  // Read every 
const unsigned long writeInterval = 3000; // Write every 1000ms

void loop() {
  if (!mb.slave()) { // Check if not currently processing a slave response
    // Read holding register from slave (address 10, ID 1)
    if (millis() - lastReadTime >= readInterval) {
      lastReadTime = millis();

      Serial.print("1:readHreg:");
      readHregTime = micros();
      mb.readHreg(SLAVE_ID, REGN_READ, datarec, datasize, cbRead);
      Serial.print(micros()-readHregTime);
      Serial.print("us for readHreg:");
      Serial.println(datarec[0]);
    }

    if (millis() - lastWriteTime >= writeInterval) {
      lastWriteTime = millis();

      datawrite[0] = millis()/1000;
      writeHregTime = micros();
      mb.writeHreg(SLAVE_ID, REGN_WRITE, datawrite, datasize, cbWrite);
      Serial.print("5:writeHreg:");
      Serial.print(micros()-writeHregTime);
      Serial.println("us for writeHreg");
    }
  }
  unsigned long  temp1= micros();
  mb.task(); // Process Modbus tasks
  long timetask = micros() - temp1;
  if(timetask > 1000){
    Serial.print("4:");
    Serial.print(timetask);
    Serial.println("us for task()");
  }
}