#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpuTop(0x68);   // Top MPU6050 at address 0x68
MPU6050 mpuBottom(0x69); // Bottom MPU6050 at address 0x69

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    mpuTop.initialize();
    mpuBottom.initialize();
    
    if (!mpuTop.testConnection()) {
        Serial.println("Failed to connect to MPU6050 (Top)");
    }
    if (!mpuBottom.testConnection()) {
        Serial.println("Failed to connect to MPU6050 (Bottom)");
    }
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

    // Print data to Serial Monitor
    Serial.println("MPU6050 Sensor Data:");
    Serial.print("Top MPU6050:  X: "); Serial.print(normAxTop);
    Serial.print("  Y: "); Serial.print(normAyTop);
    Serial.print("  Z: "); Serial.println(normAzTop);

    Serial.print("Bottom MPU6050:  X: "); Serial.print(normAxBottom);
    Serial.print("  Y: "); Serial.print(normAyBottom);
    Serial.print("  Z: "); Serial.println(normAzBottom);

    Serial.println("----------------------------------");

    delay(500); // Wait before next read
}
