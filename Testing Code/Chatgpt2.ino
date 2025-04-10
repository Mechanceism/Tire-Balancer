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

// PID constants
float Kp = 1.0, Ki = 0.5, Kd = 0.1;
float prevErrorPitch = 0, prevErrorRoll = 0;
float integralPitch = 0, integralRoll = 0;

// Angle limit
const float maxAngle = 30.0; 

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

    // Apply angle limit to prevent PID from accumulating past frame limits
    if (abs(errorPitch) > maxAngle) {
        errorPitch = (errorPitch > 0) ? maxAngle : -maxAngle;
        integralPitch = 0; // Prevent integral wind-up
    }

    if (abs(errorRoll) > maxAngle) {
        errorRoll = (errorRoll > 0) ? maxAngle : -maxAngle;
        integralRoll = 0; // Prevent integral wind-up
    }

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
    controlActuator(UP1, DOWN1, outputPitch);
    controlActuator(UP2, DOWN2, outputRoll);

    Serial.println(outputPitch);
    Serial.println(outputRoll);

    delay(50);
}

void controlActuator(int upPWM, int downPWM, float output) {
    int speed = constrain(abs(output) * 10, 0, 255);  // Scale output
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
