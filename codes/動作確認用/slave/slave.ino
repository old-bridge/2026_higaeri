#include <ModbusRTU.h>


const int datasize = 50;
#define SLAVE_ID 1
#define REGN_READ 10  // Register for reading (value 100)
#define REGN_WRITE 100 // Register for writing (value 77 from master)
#define BAUDRATE 5000000

ModbusRTU mb;
HardwareSerial MySerial0(0);

void setup() {
  Serial.begin(BAUDRATE); // For debugging
  MySerial0.begin(BAUDRATE, SERIAL_8N1, -1, -1); // Default RX/TX pins
  mb.begin(&MySerial0,D5);
  mb.slave(SLAVE_ID); // Set as slave with ID 1
  mb.addHreg(REGN_READ, 100,datasize); // Initialize read register at address 10 with value 100
  mb.addHreg(REGN_WRITE, 0,datasize);  // Initialize write register at address 11 with value 0
  Serial.println("Slave initialized, waiting for requests...");
}

void loop() {
  unsigned long  temp1= micros();
  mb.task(); // Process Modbus tasks
  long timetask = micros() - temp1;
  if(timetask > 1000){
    Serial.print("2/6:");
    Serial.print(timetask);
    Serial.println("us for task()");
  }
  // Print the value of the write register when it changes
  static uint16_t lastValue = 0;
  uint16_t currentValue = mb.Hreg(REGN_WRITE);
  if (currentValue != lastValue) {
    Serial.print("Received value in register 11: ");
    Serial.println(currentValue);
    mb.Hreg(REGN_READ,currentValue+1);
    lastValue = currentValue;
  }
}