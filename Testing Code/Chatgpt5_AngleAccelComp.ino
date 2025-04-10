#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpuTop(0x68);   // MPU6050 at address 0x68
MPU6050 mpuBottom(0x69); // MPU6050 at address 0x69

// Define motor driver pins for Actuator 1
const int UP1 = 9;   // Up PWM for Actuator 1
const int DOWN1 = 10; // Down PWM for Actuator 1

// Define motor driver pins for Actuator 2
const int UP2 = 5;   // Up PWM for Actuator 2
const int DOWN2 = 6; // Down PWM for Actuator 2

// Complementary Filter constant (Adjustable for filtering)
const float alpha = 0.85;  // Higher = more gyroscope influence, lower = more accelerometer influence

// Time tracking
unsigned long prevTime = 0;

// Error limits
const float deadZone = 2.0;  // Ignore small errors below 2 degrees
const float maxSpeed = 100;  // Set a speed limit for actuators (0-255)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  mpuTop.initialize();
  mpuBottom.initialize();
  
  pinMode(UP1, OUTPUT);
  pinMode(DOWN1, OUTPUT);
  pinMode(UP2, OUTPUT);
  pinMode(DOWN2, OUTPUT);
}

void loop() {
  static float pitchTop = 0, rollTop = 0;
  int16_t axTop, ayTop, azTop, gxTop, gyTop, gzTop;
  int16_t axBottom, ayBottom, azBottom, gxBottom, gyBottom, gzBottom;

  // Read acceleration and gyroscope data
  mpuTop.getMotion6(&axTop, &ayTop, &azTop, &gxTop, &gyTop, &gzTop);
  mpuBottom.getMotion6(&axBottom, &ayBottom, &azBottom, &gxBottom, &gyBottom, &gzBottom);

  // Normalize acceleration values
  float normAxTop = axTop / 16384.0;
  float normAyTop = ayTop / 16384.0;
  float normAzTop = azTop / 16384.0;

  // Calculate pitch and roll using accelerometer
  float accelPitchTop = atan2(normAxTop, normAzTop) * 180 / PI;
  float accelRollTop = atan2(normAyTop, normAzTop) * 180 / PI;

  // Get time step
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Convert gyroscope readings from raw to deg/sec
  float gyroPitchRateTop = gxTop / 131.0;
  float gyroRollRateTop = gyTop / 131.0;

  // Complementary filter: combines accelerometer and gyroscope data
  pitchTop = alpha * (pitchTop + gyroPitchRateTop * dt) + (1 - alpha) * accelPitchTop;
  rollTop = alpha * (rollTop + gyroRollRateTop * dt) + (1 - alpha) * accelRollTop;

  // Calculate errors (desired Z-axis should be 1, meaning zero tilt)
  float errorPitch = -pitchTop;
  float errorRoll = -rollTop;

  // Apply dead zone to avoid small oscillations
  if (abs(errorPitch) < deadZone) errorPitch = 0;
  if (abs(errorRoll) < deadZone) errorRoll = 0;

  // Actuator control (scaling angle to speed)
  controlActuator(UP1, DOWN1, errorPitch);
  controlActuator(UP2, DOWN2, errorRoll);

  // Output readings
  Serial.print("Pitch: "); Serial.print(pitchTop);
  Serial.print(" | Roll: "); Serial.println(rollTop);

  delay(50);
}

void controlActuator(int upPWM, int downPWM, float output) {
  int speed = constrain(abs(output) * 5, 0, maxSpeed);  // Reduce scaling factor
  if (output > 0) {
      analogWrite(upPWM, speed);
      analogWrite(downPWM, 0);
  } else if (output < 0) {
      analogWrite(upPWM, 0);
      analogWrite(downPWM, speed);
  } else {
      analogWrite(upPWM, 0);
      analogWrite(downPWM, 0);
  }
}
