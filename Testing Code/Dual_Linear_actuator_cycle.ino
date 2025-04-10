// Define motor driver pins for Motor 1
const int RPWM1 = 9;  // Forward PWM for Motor 1
const int LPWM1 = 10; // Reverse PWM for Motor 1

// Define motor driver pins for Motor 2
const int RPWM2 = 5;  // Forward PWM for Motor 2
const int LPWM2 = 6;  // Reverse PWM for Motor 2

int maxSpeed1 = 255; // Default max speed for Motor 1
int maxSpeed2 = 255; // Default max speed for Motor 2

void setup() {
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    
    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);
    
    Serial.begin(9600); // Initialize serial communication
}

void loop() {
    // Check for serial input
    if (Serial.available() > 0) {
        int newSpeed1, newSpeed2;
        newSpeed1 = Serial.parseInt(); // Read first number
        newSpeed2 = Serial.parseInt(); // Read second number
        
        if (newSpeed1 > 0 && newSpeed1 <= 255) {
            maxSpeed1 = newSpeed1; // Update max speed for Motor 1
        }
        if (newSpeed2 > 0 && newSpeed2 <= 255) {
            maxSpeed2 = newSpeed2; // Update max speed for Motor 2
        }
        
        Serial.print("Max speed set to: Motor 1 -> ");
        Serial.print(maxSpeed1);
        Serial.print(", Motor 2 -> ");
        Serial.println(maxSpeed2);
    }

    // Run Motor 1 forward
    analogWrite(RPWM1, maxSpeed1);
    analogWrite(LPWM1, 0);
    
    // Run Motor 2 forward
    analogWrite(RPWM2, maxSpeed2);
    analogWrite(LPWM2, 0);
    delay(2000); // Run for 2 seconds
    
    // Stop motors
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
    delay(500); // Pause for 0.5 seconds

    // Run Motor 1 in reverse
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, maxSpeed1);
    
    // Run Motor 2 in reverse
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, maxSpeed2);
    delay(2000); // Run for 2 seconds
    
    // Stop motors
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
    delay(500); // Pause for 0.5 seconds
}
