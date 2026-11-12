#define TP_RST 25
#define I2C_SDA 33
#define I2C_SCL 32

#include <Wire.h>

void setup() {
  pinMode(TP_RST, OUTPUT);

  digitalWrite(TP_RST, LOW);  // Hold in reset
  delay(50);                  // 50ms hold
  digitalWrite(TP_RST, HIGH); // Release
  delay(100);                 // 100ms wait for boot

  Wire.begin(I2C_SDA, I2C_SCL); 
  Serial.begin(115200);
  Serial.println("\nStarting I2C Scanner...");
}

void loop() {
  byte error, address;
  int devices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      devices++;
    }
  }
  if (devices == 0)
    Serial.println("No I2C devices found.\n");
  else
    Serial.println("Done.\n");

  delay(5000);
}
