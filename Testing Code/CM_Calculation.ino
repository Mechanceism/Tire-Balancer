int RimD = 0; //Rim Diameter (m)
int TreadW = 0;  //Tread Width (m)
int SideWallRatio = 0; //Side Wall Ratio (%)
int SideWallHeight = 0;  //Side Wall Height = Tread Width (m) * Side Wall Ratio (%)
int TotalRadius = 0;  //Total Radius = Side Wall Height (m) + RimD/2 (m)
int HalfSideWallRadius = 0; //Radius to half the sidewall, Side Wall Height/2 (m) + Rim Diameter/2 (m)

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

const float pi = 3.1415;

void setup() {

}

void loop() {
  ReadTireData();
  ReadMass();
  solveCentroidAndWeight();

}

void solveCentroidAndWeight(){
  XCentroid = ((WA*XA)+(WB*XB)+(WC*XC))/(WA+WB+WC);
  YCentroid = ((WA*YA)+(WB*YB)+(WC*YC))/(WA+WB+WC);
  CMRadiusNotScaled = sqrt(XCentroid*XCentroid + YCentroid*YCentroid);
  CMRadius = CMRadiusNotScaled * HalfSideWallRadius;
  CMAngle = tan(YCentroid / XCentroid);
  WeightAngle = CMAngle + pi;
  WeightMass = 2*CMRadius*(WA+WB+WC)/RimD;  //Sigma(Moment) = CMRadius(WA+WB+WC) - WeightMass(RimD/2)
}

void ReadMass(){
  WA = 0;
  WB = 0;
  WC = 0;
}

void ReadTireData(){
  RimD = 0;
  TreadW = 0;
  SideWallRatio = 0;
  SideWallHeight = TreadW * SideWallRatio;
  TotalRadius = 0;
  HalfSideWallRadius = 0;
}
