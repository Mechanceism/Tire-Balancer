// Arduino Nano - Sender
void setup() {
  Serial.begin(9600); // Start hardware serial
}

void loop() {
  Serial.println("Hello from Nano!");
  delay(1000); // Send every 1 second
}
