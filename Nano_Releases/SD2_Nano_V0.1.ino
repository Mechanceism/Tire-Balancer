//--------------------------------------------------
// start Level (Initialize)

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
  int LDcycle = 0;

  // Averaged Angles
  #define NUM_ERROR_CYCLES 3  // Change this to 4, 5, etc. as needed
  float errorPitchHistory[NUM_ERROR_CYCLES] = {0};
  float errorRollHistory[NUM_ERROR_CYCLES] = {0};
  int errorIndex = 0;
  float errorPitchAvg = 0;
  float errorRollAvg = 0;

// end Level (Initialize)
//------------------------------------------------------------
// start CM_Calculation (Initialize)

  #include "HX711.h"

  // HX711 circuit wiring for three load cells
  const int LOADCELL_DOUT_PINS[] = {2, 7, 11};  // Data pins
  const int LOADCELL_SCK_PINS[] = {3, 8, 12};   // Clock pins

  HX711 scales[3];

  // Variables to store readings
  long Scale1 = 0, Scale2 = 0, Scale3 = 0;

  //Scale Calibration
  bool requestCalibrate = false;
  float Scale1Reference = 0;
  float Scale2Reference = 0;
  float Scale3Reference = 0;
  float Scale1Calibration = 200000;
  float Scale2Calibration = 200000;
  float Scale3Calibration = 200000;
  float TestMass = 5;

  float RimD = 0; //Rim Diameter (m)
  float TreadW = 0;  //Tread Width (m)
  float SideWallRatio = 0; //Side Wall Ratio (%)
  float SideWallHeight = 0;  //Side Wall Height = Tread Width (m) * Side Wall Ratio (%)
  float TotalRadius = 0;  //Total Radius = Side Wall Height (m) + RimD/2 (m)
  float HalfSideWallRadius = 0; //Radius to half the sidewall, Side Wall Height/2 (m) + Rim Diameter/2 (m)
  float RimDInput = 0;
  float TreadWInput = 0;
  float SideWallRatioInput = 0;

  float WA = 0;
  float WB = 0;
  float WC = 0;
  const float XA = -sqrt(3/2);
  const float XB = 0;
  const float XC = sqrt(3/2);
  const float YA = -1/2;
  const float YB = 1;
  const float YC = -1/2;

  float XCentroid = 0;
  float YCentroid = 0;
  float CMRadiusNotScaled = 0;
  float CMRadius = 0;
  float CMAngle = 0;
  float WeightAngle = 0;
  float WeightMass = 0;
  float WeightMassConvert = 0;
  float TotalWeight = 0;
  float XWeight = 0;
  float YWeight = 0;

  const float pi = 3.141592653;

  // Debug enable/disable
  bool doBD = false;
  int BDcycle = 0;

// end CM_Calculation (Initialize)
//-----------------------------------------------------------

void setup() {
  //---------------------------------------------------------
  // start Level (SETUP)

  Serial.begin(9600);
  Wire.begin();
  
  mpuTop.initialize();
  mpuBottom.initialize();
  
  pinMode(UP1, OUTPUT);
  pinMode(DOWN1, OUTPUT);
  pinMode(UP2, OUTPUT);
  pinMode(DOWN2, OUTPUT);

  // end Level (SETUP)
  //------------------------------------------------------------
  // start CM_Calculation (SETUP)

  for (int i = 0; i < 3; i++) {
    scales[i].begin(LOADCELL_DOUT_PINS[i], LOADCELL_SCK_PINS[i]);
  }
  // end CM_Calculation (SETUP)
  //-----------------------------------------------------------
}

void loop() {
  //---------------------------------------------------------
  // start Level (LOOP)

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

  if (LDcycle > 0) {
    doLD = true;
    LDcycle--;
  } else if(LDcycle == 0) {
    doLD = false;
    LDcycle--;
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

  // end Level (LOOP)
  //------------------------------------------------------------
  // start CM_Calculation (LOOP)

  checkSerialInput();
  ReadTireData();
  ReadMass();
  solveCentroidAndWeight();

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

  if (BDcycle > 0) {
    doBD = true;
    BDcycle--;
  } else if(BDcycle == 0) {
    doBD = false;
    BDcycle--;
  }

  // Print values
  if(doBD == true) {
  Serial.print("Scales (1,2,3): "); Serial.print(WA);
  Serial.print(", "); Serial.print(WB);
  Serial.print(", "); Serial.print(WC);
  Serial.print(" | Centroid (X&Y): "); Serial.print(XCentroid);
  Serial.print(",  "); Serial.print(YCentroid);
  Serial.print(" | WeightAngle: "); Serial.print(WeightAngle);
  Serial.print(" | WeightMass: "); Serial.print(WeightMass);
  Serial.print(" | XWeight: "); Serial.print(XWeight);
  Serial.print(" | YWeight: "); Serial.print(YWeight);
  Serial.print(" | TotalWeight: "); Serial.println(TotalWeight);
  }
  
  delay(100);
  // end CM_Calculation (LOOP)
  //-----------------------------------------------------------
}

//--------------------------------------------------
// start Level & CM_Calculation (Functions)

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
    } else if (input.equalsIgnoreCase("LDon")) {
      doLD = true;
      return;
    } else if (input.equalsIgnoreCase("LDoff")) {
      doLD = false;
      return;
    } else if (input.equalsIgnoreCase("BDon")) {
      doBD = true;
      return;
    } else if (input.equalsIgnoreCase("BDoff")) {
      doBD = false;
      return;
    } else if (input.equalsIgnoreCase("calibrate")) {
      requestCalibrate = true;
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
          else if (currentKey.equalsIgnoreCase("LDcycle")) {
          LDcycle = value;
          }
          else if (currentKey.equalsIgnoreCase("BDcycle")) {
          BDcycle = value;
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

//--------------------------------------------------
// start Level (Functions)

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

// end Level (Functions)
//------------------------------------------------------------
// start CM_Calculation (Functions)

void solveCentroidAndWeight(){
  //XCentroid = ((WA*XA)+(WB*XB)+(WC*XC))/(WA+WB+WC);
  //YCentroid = ((WA*YA)+(WB*YB)+(WC*YC))/(WA+WB+WC);
  XCentroid = WA*cosf(pi*7/6) + WC*cosf(pi*-1/6);
  YCentroid = WA*sinf(pi*7/6) + WB*sinf(pi/2) + WC*sinf(pi*-1/6);
  CMRadiusNotScaled = sqrtf(XCentroid*XCentroid + YCentroid*YCentroid);
  CMRadius = CMRadiusNotScaled * HalfSideWallRadius;
  CMAngle = atan2f(YCentroid, XCentroid);
  WeightAngle = (CMAngle + pi)*(180/pi);
  TotalWeight = WA+WB+WC;

  XWeight = WA*cosf(CMAngle + pi*7/6) + WB*cosf(CMAngle + pi/2) + WC*cosf(CMAngle + pi*-1/6);
  YWeight = WA*sinf(CMAngle + pi*7/6) + WB*sinf(CMAngle + pi/2) + WC*sinf(CMAngle + pi*-1/6);

  WeightMass = 2*CMRadius*(sqrtf(XWeight*XWeight + YWeight*YWeight))/RimD;  // Sigma(Moment) = 0 = CMRadius(TotalWeight) - WeightMass(RimD/2)
  WeightMassConvert = WeightMass*(35.274/1); // kilograms to OZs
}

void ReadMass(){
  if(requestCalibrate == true){
    requestCalibrate = false;
    Scale1Reference = Scale1;
    Scale2Reference = Scale2;
    Scale3Reference = Scale3;
  }
  
  WA = abs((Scale1 - Scale1Reference)/(Scale1Calibration))+TestMass;
  WB = abs((Scale2 - Scale2Reference)/(Scale2Calibration))+TestMass;
  WC = abs((Scale3 - Scale3Reference)/(Scale3Calibration))+TestMass;
}

void ReadTireData(){
  RimDInput = 15;
  TreadWInput = 215;
  SideWallRatioInput = 65;

  RimD = RimDInput*(2.54/100); //  Converts Inches to Meters
  TreadW = TreadWInput/1000; //  Converts mm to Meters
  SideWallRatio = SideWallRatioInput/100; //  Converts percent to decimal
  SideWallHeight = TreadW * SideWallRatio;
  TotalRadius = RimD/2 + SideWallHeight;
  HalfSideWallRadius = RimD/2 + SideWallHeight/2;
}

// end CM_Calculation (Functions)
//-----------------------------------------------------------