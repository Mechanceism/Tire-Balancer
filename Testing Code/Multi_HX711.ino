#include "HX711.h"

// HX711 circuit wiring for three load cells
const int LOADCELL_DOUT_PINS[] = {4, 6, 8};  // Data pins
const int LOADCELL_SCK_PINS[] = {5, 7, 9};   // Clock pins

HX711 scales[3];

// Variables to store readings
long Scale1 = 0, Scale2 = 0, Scale3 = 0;

void setup() {
  Serial.begin(9600);
  
  for (int i = 0; i < 3; i++) {
    scales[i].begin(LOADCELL_DOUT_PINS[i], LOADCELL_SCK_PINS[i]);
  }
}

void loop() {
  long readings[3] = {Scale1, Scale2, Scale3};  // Array for storing values

  for (int i = 0; i < 3; i++) {
    if (scales[i].is_ready()) {
      readings[i] = scales[i].read();
    } else {
      Serial.print("HX711[");
      Serial.print(i);
      Serial.println("] not found.");
    }
  }

  // Assign values to named variables
  Scale1 = readings[0];
  Scale2 = readings[1];
  Scale3 = readings[2];

  // Print values
  Serial.print("Scale1: "); Serial.println(Scale1);
  Serial.print("Scale2: "); Serial.println(Scale2);
  Serial.print("Scale3: "); Serial.println(Scale3);

  delay(1000);
}
