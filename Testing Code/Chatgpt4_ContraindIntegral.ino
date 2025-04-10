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

// Tuned PID constants
float Kp = 0.4, Ki = 0.1, Kd = 0.25;
float prevErrorPitch = 0, prevErrorRoll = 0;
float integralPitch = 0, integralRoll = 0;

// Angle and integral limits
const float maxAngle = 30.0; 
const float maxIntegral = 20.0; // Prevents integral wind-up

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
  int16_t axTop, ayTop, azTop, axBottom, ayBottom, azBottom;

  // Read acceleration data
  mpuTop.getAcceleration(&axTop, &ayTop, &azTop);
  mpuBottom.getAcceleration(&axBottom, &ayBottom, &azBottom);

  // Normalize values (MPU6050 raw data range is -16384 to 16384, where 16384 represents 1g)
  float normAxTop = axTop / 16384.0;
  float normAyTop = ayTop / 16384.0;
  float normAzTop = azTop / 16384.0;

  float normAxBottom = axBottom / 16384.0;
  float normAyBottom = ayBottom / 16384.0;
  float normAzBottom = azBottom / 16384.0;
  
  // Calculate tilt angles of top plate
  float pitchTop = atan2(normAxTop, normAzTop) * 180 / PI;
  float rollTop = atan2(normAyTop, normAzTop) * 180 / PI;

  // Desired top plate Z should be 1 (level)
  float errorPitch = -pitchTop;
  float errorRoll = -rollTop;

  // Apply angle limit to prevent exceeding frame limits
  errorPitch = constrain(errorPitch, -maxAngle, maxAngle);
  errorRoll = constrain(errorRoll, -maxAngle, maxAngle);

  // Prevent integral wind-up
  integralPitch = constrain(integralPitch + errorPitch, -maxIntegral, maxIntegral);
  integralRoll = constrain(integralRoll + errorRoll, -maxIntegral, maxIntegral);

  // PID calculations
  float dErrorPitch = errorPitch - prevErrorPitch;
  float dErrorRoll = errorRoll - prevErrorRoll;

  float outputPitch = Kp * errorPitch + Ki * integralPitch + Kd * dErrorPitch;
  float outputRoll = Kp * errorRoll + Ki * integralRoll + Kd * dErrorRoll;

  prevErrorPitch = errorPitch;
  prevErrorRoll = errorRoll;

  // Scale output for smooth speed control
  controlActuator(UP1, DOWN1, errorPitch);
  controlActuator(UP2, DOWN2, errorRoll);

  Serial.print("Pitch Output: "); Serial.print(outputPitch);
  Serial.print(" | Roll Output: "); Serial.println(outputRoll);

  delay(50);
}

void controlActuator(int upPWM, int downPWM, float output) {
  // Smooth motion mapping using a logarithmic scale
  int speed = constrain(map(abs(output), 0, maxAngle, 50, 255), 0, 255);

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
