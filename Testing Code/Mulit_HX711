#include "HX711.h"

// HX711 circuit wiring for three load cells
const int LOADCELL_DOUT_PINS[] = {4, 6, 8};  // Data pins
const int LOADCELL_SCK_PINS[] = {5, 7, 9};   // Clock pins

HX711 scales[3];

void setup() {
  Serial.begin(57600);
  
  for (int i = 0; i < 3; i++) {
    scales[i].begin(LOADCELL_DOUT_PINS[i], LOADCELL_SCK_PINS[i]);
  }
}

void loop() {
  for (int i = 0; i < 3; i++) {
    if (scales[i].is_ready()) {
      long reading = scales[i].read();
      Serial.print("HX711[");
      Serial.print(i);
      Serial.print("] reading: ");
      Serial.println(reading);
    } else {
      Serial.print("HX711[");
      Serial.print(i);
      Serial.println("] not found.");
    }
  }

  delay(1000);
}
