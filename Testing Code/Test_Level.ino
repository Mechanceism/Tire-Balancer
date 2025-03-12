//X axis: Pitch, axis between solid and actuators
//Y axis: Roll, axis between actuators

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpuTop;
MPU6050 mpuBottom;

// Actuator control pins
const int actuator1UpPin = 3;
const int actuator1DownPin = 5;
const int actuator1PWMPin = 6;

const int actuator2UpPin = 9;
const int actuator2DownPin = 10;
const int actuator2PWMPin = 11;

int actuator1PWM = 0;
int actuatorPWM = 0;
int MaxAllowedBaseAngle = 20;
int MaxAllowedPlateAngle = 20;
int MaxAllowedZTwistAngle = 5;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    mpuTop.initialize();
    mpuBottom.initialize();
    
    pinMode(actuator1Up, OUTPUT);
    pinMode(actuator1Down, OUTPUT);
    pinMode(actuator1PWM, OUTPUT);
    
    pinMode(actuator2Up, OUTPUT);
    pinMode(actuator2Down, OUTPUT);
    pinMode(actuator2PWM, OUTPUT);
}

void loop() {
    // Read accelerometer data
    int16_t axTop, ayTop, azTop, axBottom, ayBottom, azBottom;
    mpuTop.getAcceleration(&axTop, &ayTop, &azTop);
    mpuBottom.getAcceleration(&axBottom, &ayBottom, &azBottom);

    //float normTopZ = azTop / 16384.0;  
    //float normBottomX = axBottom / 16384.0;
    //float normBottomY = ayBottom / 16384.0;

    if (axBottom >= MaxAllowedBaseAngle || ayBottom >= MaxAllowedBaseAngle || azBottom >= MaxAllowedBaseAngle) {
      serial.Print('Error: Base past allowed angle');
      return;
    }

    axdiff = (axTop - axBottom)*cos(ayBottom)
    aydiff = (ayTop - ayBottom)*cos(axBottom)
    azdiff = (azTop - azBottom) //if more abs(morethan 0) then parallelogramming

    if (axdiff >= MaxAllowedPlateAngle || aydiff >= MaxAllowedPlateAngle) {
      serial.Print('Error: Top Plate past allowed angle');
      return;
    }
    
    if (azdiff >= MaxAllowedZTwistAngle) {
      serial.Print('Error: Frame parallelogram past allowed angle');
      return;
    }

    actuator1YAngle = map(ayTop, -MaxAllowedPlateAngle, MaxAllowedPlateAngle, 255, -255);
    actuator2YAngle = map(ayTop, -MaxAllowedPlateAngle, MaxAllowedPlateAngle, 255, -255);

    actuator1PWM = actuator1YAngle + map(axTop, -MaxAllowedPlateAngle, MaxAllowedPlateAngle, 255, -255);
    actuator2PWM = actuator2YAngle + map(axTop, -MaxAllowedPlateAngle, MaxAllowedPlateAngle, 255, -255);
    
    // Actuator control (mapping angles to movement)
    controlActuator(actuator1UpPin, actuator1DownPin, actuator1PWMPin, actuator1PWM);
    controlActuator(actuator2UpPin, actuator2DownPin, actuator2PWMPin, actuator2PWM);

    //delay(50);
}

void controlActuator(int upPin, int downPin, int pwmPin, float output) {
    int speed = constrain(abs(output) * 10, 0, 255);  // Scale output
    if (output > 0) {
        digitalWrite(upPin, HIGH);
        digitalWrite(downPin, LOW);
    } else if (output < 0) {
        digitalWrite(upPin, LOW);
        digitalWrite(downPin, HIGH);
    } else {
        digitalWrite(upPin, LOW);
        digitalWrite(downPin, LOW);
    }
    analogWrite(pwmPin, speed);
}
