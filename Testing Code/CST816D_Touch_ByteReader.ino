#include <Wire.h>

#define I2C_SDA 33
#define I2C_SCL 32
#define CST820_ADDR 0x15

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
}

void loop() {
  Wire.beginTransmission(CST820_ADDR);
  Wire.write(0x00); // Start at 0x00
  Wire.endTransmission(false);

  // Try reading a lot of bytes
  Wire.requestFrom(CST820_ADDR, 16);

  Serial.print("Bytes Available: ");
  Serial.println(Wire.available());

  while (Wire.available()) {
    uint8_t val = Wire.read();
    Serial.print("0x");
    Serial.print(val, HEX);
    Serial.print(" ");
  }
  Serial.println();

  delay(200);
}
