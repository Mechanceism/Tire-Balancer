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

bool doBD = false;

void setup() {
  Serial.begin(9600);
  
  for (int i = 0; i < 3; i++) {
    scales[i].begin(LOADCELL_DOUT_PINS[i], LOADCELL_SCK_PINS[i]);
  }
}

void loop() {
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
}

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

void checkSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();  // Remove any trailing spaces or newlines
        
        if (input.equalsIgnoreCase("CALIBRATE")) {
            requestCalibrate = true;
            Serial.println("CALIBRATE");
        }
    }
}