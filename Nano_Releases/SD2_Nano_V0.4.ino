//--------------------------------------------------
// start Level (Initialize)

  #include <Wire.h>
  #include <MPU6050.h>
  #include <ctype.h>

  MPU6050 mpuTop(0x68);   // MPU6050 at address 0x68
  MPU6050 mpuBottom(0x69); // MPU6050 at address 0x69

  // Define motor driver pins for Actuators
  const int UP1 = 9;   // Up PWM for Actuator 1
  const int DOWN1 = 10; // Down PWM for Actuator 1
  const int UP2 = 5;   // Up PWM for Actuator 2
  const int DOWN2 = 6; // Down PWM for Actuator 2

  // Complementary Filter constant
  float alpha = 0.50;  // Balance between gyro & accel

  // Time tracking
  unsigned long prevTime = 0;

  // Tuning parameters
  const float deadZone = 0.0;  // Ignore small errors (0 is unused)
  const float MaxBaseAngle = 13.0; // Max Base Angle in degrees
  float minSpeed = 50; // Minimum speed of actuators (out of 255)
  float maxSpeed = 100; // Maximum speed of actuators (out of 255)
  float rollOffset = 0; // (in degrees)
  float pitchOffset = 2; // (in degrees)

  // Auto Angle Level Stop
  bool requestLevel = false;
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

  // Voltage Compensation for weight
  float MassCompensationState = false;

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
  float WAB = 0;
  float WBC = 0;

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
//------------------------------------------------------------
// start Balance (Initialize)

  const int numSamples = 10; // Number of samples for the WeightAngle and WeightMass average for balancing
  float weightAngleBuffer[numSamples];
  float weightMassBuffer[numSamples];

  int bufferIndex = 0;

  float WeightAngleAvg = 0.0;
  float WeightMassAvg = 0.0;

  bool BalanceState = false;    // External trigger
  bool isCollecting = false;    // Internal flag to track progress
  int collectedSamples = 0;     // Counts how many valid samples have been collected

// end Balance (Initialize)
//-----------------------------------------------------------
// Start Laser (Initialize)
  #include <math.h>
  
  const int servoPin = 4;
  int Laser1 = A2;
  int Laser2 = A3;

  int degree0 = 550;
  int degree180 = 2260;
  float LaserOffset = 82;

  volatile int pulseWidth = degree0;  // In microseconds
  volatile bool doPulse = false;

  bool laserstate = false;

// End Laser (Initialize)
//-----------------------------------------------------------
// Start Counts Per Second (Initialize)

  unsigned long loopCount = 0;
  unsigned long lastPrintTime = 0;
  bool CountState = false;

// End Counts Per Second (Initialize)
//-----------------------------------------------------------

void setup() {
  //---------------------------------------------------------
  // start Level (SETUP)

  Serial.begin(9600);
  Wire.begin();
  
  mpuTop.initialize();
  mpuBottom.initialize();

  mpuBottom.setXAccelOffset(-414); //Set your accelerometer offset for axis X
  mpuBottom.setYAccelOffset(429); //Set your accelerometer offset for axis Y
  mpuBottom.setZAccelOffset(4791); //Set your accelerometer offset for axis Z
  
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
  // start Balance (SETUP)

  //Empty

  // end Balance (SETUP)
  //-----------------------------------------------------------
  // start Laser (SETUP)

  pinMode(servoPin, OUTPUT);
  Serial.begin(9600);
  setupServoTimer2();

  pinMode(Laser1, OUTPUT);
  pinMode(Laser2, OUTPUT);
  digitalWrite(Laser1, LOW);
  digitalWrite(Laser2, LOW);

  // end Laser (SETUP)
  //-----------------------------------------------------------
}

void loop() {
  //---------------------------------------------------------
  // start Level (LOOP)

  //float pitchTop = 0, rollTop = 0;
  int16_t axTop, ayTop, azTop, gxTop, gyTop, gzTop;
  int16_t axBottom, ayBottom, azBottom;

  // Read acceleration and gyroscope data
  mpuTop.getMotion6(&axTop, &ayTop, &azTop, &gxTop, &gyTop, &gzTop);
  mpuBottom.getAcceleration(&axBottom, &ayBottom, &azBottom);

  // Normalize acceleration values
  float normAxTop = axTop / 16384.0;
  float normAyTop = ayTop / 16384.0;
  float normAzTop = azTop / 16384.0;
  float normAxBottom = axBottom / 16384.0;
  float normAyBottom = ayBottom / 16384.0;
  float normAzBottom = azBottom / 16384.0;

  // Calculate pitch and roll using accelerometer
  float accelPitchTop = atan2(normAxTop, 1) * 180 / PI;
  float accelRollTop = atan2(normAyTop, 1) * 180 / PI;
  float accelPitchBottom = atan2(normAxBottom, 1) * 180 / PI;
  float accelRollBottom = atan2(normAyBottom, 1) * 180 / PI;

  float BaseAngle = sqrt(accelPitchBottom*accelPitchBottom + accelRollBottom*accelRollBottom);

  if(BaseAngle >= MaxBaseAngle){
    Serial.println("Error: Base too unlevel");
  }

  // %Error Gravity Calc
  float IsPitchTrans = sqrtf(normAxTop*normAxTop + normAzTop*normAzTop);
  float IsRollTrans = sqrtf(normAyTop*normAyTop + normAzTop*normAzTop);

  // Get time step
  unsigned long currentTimeLevel = millis();
  float dt = (currentTimeLevel - prevTime) / 1000.0;
  prevTime = currentTimeLevel;

  // Convert gyroscope readings from raw to deg/sec
  float gyroPitchRateTop = gxTop / 131.0;
  float gyroRollRateTop = gyTop / 131.0;

  // Complementary filter: combines accelerometer and gyroscope data
  pitchTop = alpha * (pitchTop + gyroPitchRateTop * dt) + (1 - alpha) * accelPitchTop;
  rollTop = alpha * (rollTop + gyroRollRateTop * dt) + (1 - alpha) * accelRollTop;

  // Calculate errors (desired Z-axis should be 1, meaning zero tilt)
  float errorPitch = -pitchTop + pitchOffset;
  float errorRoll = -rollTop + rollOffset;

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
  if(requestLevel == true) {
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
  Serial.print("Pitch: "); Serial.print(pitchTop-pitchOffset);
  Serial.print(" | Roll: "); Serial.print(rollTop-rollOffset);
  Serial.print(" | IsPitchTrans: "); Serial.print(IsPitchTrans);
  Serial.print(" | IsRollTrans: "); Serial.print(IsRollTrans);
  Serial.print(" | BaseAngle: "); Serial.println(BaseAngle);
  }

  checkSerialInput();
  checkPitchStability();

  // end Level (LOOP)
  //------------------------------------------------------------
  // start CM_Calculation (LOOP)

  //checkSerialInput();
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
  // start Balance (LOOP)

  if(BalanceState && requestLevel) {
    Serial.println("Error: Cannot Balance while Leveling");
    BalanceState = false;
  }

  if (BalanceState && !isCollecting && !requestLevel) {  // If BalanceState was triggered, start collecting
    isCollecting = true;
    collectedSamples = 0;
    bufferIndex = 0; // Start filling from beginning if you want a fresh batch
  }

  if (isCollecting) { // Only collect if BalanceState is active
    weightAngleBuffer[bufferIndex] = WeightAngle;
    weightMassBuffer[bufferIndex] = WeightMass;

    bufferIndex = (bufferIndex + 1) % numSamples;
    collectedSamples++;

    if (collectedSamples >= numSamples) {
      float angleSum = 0; // Enough samples collected, calculate average
      float massSum = 0;

      for (int i = 0; i < numSamples; i++) {
        angleSum += weightAngleBuffer[i];
        massSum += weightMassBuffer[i];
      }

      WeightAngleAvg = angleSum / numSamples;
      WeightMassAvg = massSum / numSamples;

      Serial.print("WeightAngleAvg"); Serial.print(WeightAngleAvg);
      Serial.print("  WeightMassAvg"); Serial.print(WeightMassAvg);
      Serial.print("  TotalWeight"); Serial.println(TotalWeight);
      Serial.println("BalanceCompleted");

      laserstate = true;
      BalanceState = false; // Reset state
      isCollecting = false;
    }
  }
  // end Balance (LOOP)
  //-----------------------------------------------------------
  // start Laser (LOOP)

  if(laserstate == true){
    LaserPosition();
  } else {
    digitalWrite(Laser1, LOW);
    digitalWrite(Laser2, LOW);
  }
  
  // end Laser (LOOP)
  //-----------------------------------------------------------
  // Start Counts Per Second (LOOP)
  
  if(CountState == true){
    loopCount++; // Count this iteration
    unsigned long currentTimeCount = millis();
    if (currentTimeCount - lastPrintTime >= 1000) { // Every 1 second
      Serial.print("Loops per second: ");
      Serial.println(loopCount);
      loopCount = 0; // Reset count for next second
      lastPrintTime = currentTimeCount; // Update timer
    }
  }
  
  // End Counts Per Second (LOOP)
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
    if (input.equalsIgnoreCase("Start")) {
      requestLevel = true;
      stableCount = 0;
      return;
    } else if (input.equalsIgnoreCase("STOP")) {
      requestLevel = false;
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
    } else if (input.equalsIgnoreCase("laseron")) {
      laserstate = true;
      return;
    } else if (input.equalsIgnoreCase("laseroff")) {
      laserstate = false;
      return;
    } else if (input.equalsIgnoreCase("lasertoggle")) {
      laserstate = !laserstate;
      return;
    } else if (input.equalsIgnoreCase("CPSon")) {
      CountState = true;
      return;
    } else if (input.equalsIgnoreCase("CPSoff")) {
      CountState = false;
      return;
    } else if (input.equalsIgnoreCase("MCon")) {
      MassCompensationState = true;
      Serial.println("Mass Compensation ON");
      return;
    } else if (input.equalsIgnoreCase("MCoff")) {
      MassCompensationState = false;
      Serial.println("Mass Compensation OFF");
      return;
    } else if (input.equalsIgnoreCase("Balance")) {
      BalanceState = true;
      Serial.println("Balance Started");
      return;
    } else if (input.equalsIgnoreCase("BalanceCancel")) {
      BalanceState = false;
      isCollecting = false;
      Serial.println("Balance Canceled");
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
          else if (currentKey.equalsIgnoreCase("RimD")) {
          RimD = value;
          }
          else if (currentKey.equalsIgnoreCase("TreadW")) {
          TreadW = value;
          }
          else if (currentKey.equalsIgnoreCase("SideWallRatio")) {
          SideWallRatio = value;
          }
          else if (currentKey.equalsIgnoreCase("TestMass")) {
          TestMass = value;
          Serial.print("TestMass: "); Serial.println(TestMass);
          }
          else if (currentKey.equalsIgnoreCase("rollOffset")) {
          rollOffset = value;
          Serial.print("rollOffset: "); Serial.println(rollOffset);
          }
          else if (currentKey.equalsIgnoreCase("pitchOffset")) {
          pitchOffset = value;
          Serial.print("pitchOffset: "); Serial.println(pitchOffset);
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

// end Level & CM_Calculation (Functions)
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

    if (requestLevel) {
        if (abs(errorPitchAvg) >= 0 && abs(errorPitchAvg) <= 1 && abs(errorRollAvg) >= 0 && abs(errorRollAvg) <= 1) {
            stableCount++;
        } else {
            stableCount = 0;  // Reset counter if out of range
        }

        if (stableCount > stabilityThreshold) {
            requestLevel = false;
            Serial.println("LevelCompleted");
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

  if(MassCompensationState == true){
    WAB = map(sqrt(WA*WA + WB*WB),0,20,1,2);
    WBC = map(sqrt(WB*WB + WC*WC),0,20,1,2);
  } else {
    WAB = 1;
    WBC= 1;
  }

  // Project input error vector onto each actuator direction
  float output1 = (inputX * dir1X + inputY * dir1Y)*WAB;  // Actuator at 150°
  float output2 = (inputX * dir2X + inputY * dir2Y)*WBC;  // Actuator at 30°

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
// Start laser (Functions)

void LaserPosition(){
  float WeightAngleAvgShift = fmod(WeightAngleAvg + LaserOffset, 360.0);
  if (WeightAngleAvgShift < 0) {
    WeightAngleAvgShift += 360.0;
  }
  if (WeightAngleAvgShift <= 180) {
  pulseWidth = map(WeightAngleAvgShift, 0,180,degree0,degree180);
  digitalWrite(Laser1, HIGH);
  digitalWrite(Laser2, LOW);
  } else {
  pulseWidth = map(WeightAngleAvgShift-180, 0,180,degree0,degree180);
  digitalWrite(Laser1, LOW);
  digitalWrite(Laser2, HIGH);
  }

  // Handle pulse outside interrupt
  if (doPulse) {
    doPulse = false;
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);  // Accurate pulse
    digitalWrite(servoPin, LOW);
  }
}

// Timer2 triggers every 20ms
void setupServoTimer2() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 249;             // 1ms tick (16MHz / 64 / 250)
  TCCR2A |= (1 << WGM21);  // CTC mode
  TCCR2B |= (1 << CS22);   // Prescaler 64
  TIMSK2 |= (1 << OCIE2A); // Enable compare match interrupt
}

ISR(TIMER2_COMPA_vect) {
  static uint8_t msCount = 0;
  msCount++;

  if (msCount >= 20) {
    msCount = 0;
    doPulse = true;  // Trigger the pulse in main loop
  }
}

// end laser (Functions)
//-----------------------------------------------------------