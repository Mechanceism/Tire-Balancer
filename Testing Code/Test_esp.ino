void setup() {
  Serial.begin(9600); // USB serial monitor
  Serial1.begin(9600, SERIAL_8N1, 1, 3); 
  // RX = 35, TX = none (-1 means unused)
  Serial.println("ESP32 Ready to receive...");
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
  }
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);
  }
}
