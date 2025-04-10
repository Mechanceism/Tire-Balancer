#include <Wire.h>
#include <MPU6050.h>
#include <ctype.h>

MPU6050 mpuTop(0x68);   // MPU6050 at address 0x68
MPU6050 mpuBottom(0x69); // MPU6050 at address 0x69

// Define motor driver pins for Actuator 1
const int UP1 = 9;   // Up PWM for Actuator 1
const int DOWN1 = 10; // Down PWM for Actuator 1

// Define motor driver pins for Actuator 2
const int UP2 = 5;   // Up PWM for Actuator 2
const int DOWN2 = 6; // Down PWM for Actuator 2

// Complementary Filter constant
float alpha = 0.00;  // Balance between gyro & accel

// Time tracking
unsigned long prevTime = 0;

// Tuning parameters
const float deadZone = 0.0;  // Ignore small errors
const float maxAngle = 30.0; // Max angle before full speed
float minSpeed = 15; // Minimum speed of actuators
float maxSpeed = 150; // Maximum speed of actuators

// Auto Angle Level Stop
bool requestBalance = false;
int stableCount = 0;
const int stabilityThreshold = 10;
float pitchTop = 0, rollTop = 0;

// Debug enable/disable
bool doLD = false;
int cycleDebug = 0;

// Averaged Angles
#define NUM_ERROR_CYCLES 3  // Change this to 4, 5, etc. as needed
float errorPitchHistory[NUM_ERROR_CYCLES] = {0};
float errorRollHistory[NUM_ERROR_CYCLES] = {0};
int errorIndex = 0;
float errorPitchAvg = 0;
float errorRollAvg = 0;

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
  //float pitchTop = 0, rollTop = 0;
  int16_t axTop, ayTop, azTop, gxTop, gyTop, gzTop;

  // Read acceleration and gyroscope data
  mpuTop.getMotion6(&axTop, &ayTop, &azTop, &gxTop, &gyTop, &gzTop);

  // Normalize acceleration values
  float normAxTop = axTop / 16384.0;
  float normAyTop = ayTop / 16384.0;
  float normAzTop = azTop / 16384.0;

  // Calculate pitch and roll using accelerometer
  float accelPitchTop = atan2(normAxTop, 1) * 180 / PI;
  float accelRollTop = atan2(normAyTop, 1) * 180 / PI;

  // %Error Gravity Calc
  float IsPitchTrans = sqrtf(normAxTop*normAxTop + normAzTop*normAzTop);
  float IsRollTrans = sqrtf(normAyTop*normAyTop + normAzTop*normAzTop);

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

  // Store current errors in history
  errorPitchHistory[errorIndex] = errorPitch;
  errorRollHistory[errorIndex] = errorRoll;

  // Compute average over last NUM_ERROR_CYCLES
  errorPitchAvg = 0;
  errorRollAvg = 0;
  for (int i = 0; i < NUM_ERROR_CYCLES; i++) {
    errorPitchAvg += errorPitchHistory[i];
    errorRollAvg += errorRollHistory[i];
  }
  errorPitchAvg /= NUM_ERROR_CYCLES;
  errorRollAvg  /= NUM_ERROR_CYCLES;

  // Update circular buffer index
  errorIndex = (errorIndex + 1) % NUM_ERROR_CYCLES;

  // Apply dead zone
  if (abs(errorPitchAvg) < deadZone) errorPitchAvg = 0;
  if (abs(errorRollAvg) < deadZone) errorRollAvg = 0;

  // Control actuators with **properly scaled speed**
  if(requestBalance == true) {
    //controlActuator(UP1, DOWN1, errorPitchAvg);
    //controlActuator(UP2, DOWN2, errorRollAvg);
    updateActuators(errorRollAvg, errorPitchAvg);
  } else{
    controlActuator(UP1, DOWN1, 0);
    controlActuator(UP2, DOWN2, 0);
  }

  if (cycleDebug > 0) {
    doLD = true;
    cycleDebug--;
  } else if(cycleDebug == 0) {
    doLD = false;
    cycleDebug--;
  }
  
  // Output readings
  if(doLD == true){
  Serial.print("Pitch: "); Serial.print(pitchTop);
  Serial.print(" | Roll: "); Serial.print(rollTop);
  Serial.print(" | IsPitchTrans: "); Serial.print(IsPitchTrans);
  Serial.print(" | IsRollTrans: "); Serial.println(IsRollTrans);
  }

  checkSerialInput();
  checkPitchStability();

  //delay(50);
}

void controlActuator(int upPWM, int downPWM, float output) {
  // **Fix: Proportional speed scaling**
  int speed = map(abs(output*output*output), 0, 10, minSpeed, maxSpeed);
  speed = constrain(speed, minSpeed, maxSpeed);  // Ensure speed stays within limits

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

void checkSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Clean up any whitespace or line endings

    if (input.length() == 0) return;

    // Handle single-word commands
    if (input.equalsIgnoreCase("START")) {
      requestBalance = true;
      stableCount = 0;
      return;
    } else if (input.equalsIgnoreCase("STOP")) {
      requestBalance = false;
      stableCount = 0;
      return;
    } else if (input.equalsIgnoreCase("debugon")) {
      doLD = true;
      return;
    } else if (input.equalsIgnoreCase("debugoff")) {
      doLD = false;
      return;
    }

    // Parse multiple command=value entries (e.g., Kp1.2 Ki0.5)
    String currentKey = "";
    String currentValue = "";
    bool parsingValue = false;

    for (int i = 0; i <= input.length(); i++) {
      char c = input.charAt(i);
      if (isAlpha(c) && !parsingValue) {
        currentKey += c;  // Still building the key
      } 
      else if ((isDigit(c) || c == '.' || c == '-') && c != '\0') {
        currentValue += c;
        parsingValue = true;
      } 
      else {
        if (currentKey.length() > 0 && currentValue.length() > 0) {
          float value = currentValue.toFloat();

          // Match known keys and assign
          if (currentKey.equalsIgnoreCase("alpha")) {
            alpha = value;
            Serial.print("Alpha: "); Serial.println(alpha);
            }
          else if (currentKey.equalsIgnoreCase("minSpeed")) {
            minSpeed = value;
            Serial.print("minSpeed: "); Serial.println(minSpeed);
            }
          else if (currentKey.equalsIgnoreCase("maxSpeed")) {
            maxSpeed = value;
            Serial.print("maxSpeed: "); Serial.println(maxSpeed);
            }
          else if (currentKey.equalsIgnoreCase("cycleDebug")) {
          cycleDebug = value;
          }
          // Add more keys here if needed

          // Reset for the next token
          currentKey = "";
          currentValue = "";
          parsingValue = false;

          if (isAlpha(c)) currentKey += c;  // This char starts next key
        }
      }
    }
  }
}

// Function to monitor pitch stability
void checkPitchStability() {

    if (requestBalance) {
        if (abs(errorPitchAvg) >= 0 && abs(errorPitchAvg) <= 1 && abs(errorRollAvg) >= 0 && abs(errorRollAvg) <= 1) {
            stableCount++;
        } else {
            stableCount = 0;  // Reset counter if out of range
        }

        if (stableCount > stabilityThreshold) {
            requestBalance = false;
            Serial.println("Balance request cleared.");
        }
    }
}

void updateActuators(float errorRollAvg, float errorPitchAvg) {
  // Actuator directions in radians
  float angle1 = radians(150);  // Actuator 1 (@150 degrees)
  float angle2 = radians(30);   // Actuator 2 (@30 degrees)

  // Unit vectors in the direction of each actuator
  float dir1X = cos(angle1);
  float dir1Y = sin(angle1);
  float dir2X = cos(angle2);
  float dir2Y = sin(angle2);

  // Input vector based on pitch and roll error (in degrees)
  float inputX = errorRollAvg;   // Roll affects X-axis
  float inputY = errorPitchAvg;  // Pitch affects Y-axis

  // Project input error vector onto each actuator direction
  float output1 = inputX * dir1X + inputY * dir1Y;  // Actuator at 150°
  float output2 = inputX * dir2X + inputY * dir2Y;  // Actuator at 30°

  // Apply outputs to actuators
  controlActuator(UP1, DOWN1, output1);
  controlActuator(UP2, DOWN2, output2);
}