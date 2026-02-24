
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(250000);
}

void loop() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission(false);

  Wire.requestFrom(0x36, 2);

  while (Wire.available()) {
    byte angle_h = Wire.read();
    byte angle_l = Wire.read();
    unsigned int angle = (0x0F & angle_h) << 8 | angle_l;
    Serial.print(angle);
    //Serial.print(angle,HEX);
    Serial.print(" ");
  }
  delay(1);


  Wire.beginTransmission(0x36);
  Wire.write(0x0B);
  Wire.endTransmission(false);

  Wire.requestFrom(0x36, 1);

  while (Wire.available()) {
    byte state = Wire.read();
    
    Serial.println(state, BIN);
  }
  
}
