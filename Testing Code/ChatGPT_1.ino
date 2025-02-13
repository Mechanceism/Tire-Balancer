#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpuTop;
MPU6050 mpuBottom;

// Actuator control pins
const int actuator1Up = 3;
const int actuator1Down = 5;
const int actuator1PWM = 6;

const int actuator2Up = 9;
const int actuator2Down = 10;
const int actuator2PWM = 11;

// PID constants
float Kp = 10.0, Ki = 0.5, Kd = 0.1;
float prevErrorPitch = 0, prevErrorRoll = 0;
float integralPitch = 0, integralRoll = 0;

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

    // Normalize accelerometer readings (gravity = 1)
    float normTopZ = azTop / 16384.0;  
    float normBottomX = axBottom / 16384.0;
    float normBottomY = ayBottom / 16384.0;
    
    // Calculate tilt angles of bottom plate
    float pitchBottom = atan2(normBottomX, normTopZ) * 180 / PI;
    float rollBottom = atan2(normBottomY, normTopZ) * 180 / PI;

    // Desired top plate Z should be 1 (level)
    float errorPitch = -pitchBottom;
    float errorRoll = -rollBottom;

    // PID calculations
    integralPitch += errorPitch;
    integralRoll += errorRoll;

    float dErrorPitch = errorPitch - prevErrorPitch;
    float dErrorRoll = errorRoll - prevErrorRoll;

    float outputPitch = Kp * errorPitch + Ki * integralPitch + Kd * dErrorPitch;
    float outputRoll = Kp * errorRoll + Ki * integralRoll + Kd * dErrorRoll;

    prevErrorPitch = errorPitch;
    prevErrorRoll = errorRoll;

    // Actuator control (mapping angles to movement)
    controlActuator(actuator1Up, actuator1Down, actuator1PWM, outputPitch);
    controlActuator(actuator2Up, actuator2Down, actuator2PWM, outputRoll);

    delay(50);
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
