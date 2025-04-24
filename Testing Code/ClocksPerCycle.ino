unsigned long loopCount = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  loopCount++; // Count this iteration

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= 1000) { // Every 1 second
    Serial.print("Loops per second: ");
    Serial.println(loopCount);

    loopCount = 0; // Reset count for next second
    lastPrintTime = currentTime; // Update timer
  }
}
