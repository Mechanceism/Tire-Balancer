// Define motor driver pins
const int RPWM = 11;  // Forward PWM
const int LPWM = 10; // Reverse PWM

int maxSpeed = 255; // Default max speed

void setup() {
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    
    Serial.begin(9600); // Initialize serial communication
}

void loop() {
    // Check for serial input
    if (Serial.available() > 0) {
        int newSpeed = Serial.parseInt();
        if (newSpeed >= 0 && newSpeed <= 255) {
            maxSpeed = newSpeed; // Update max speed if valid
            //Serial.print("Max speed set to: ");
            Serial.println(maxSpeed);
        }
    }

    // Run forward
    analogWrite(RPWM, maxSpeed); // Use max speed
    analogWrite(LPWM, 0);
    delay(2000); // Run for 2 seconds
    
    // Stop motor
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    delay(500); // Pause for 0.5 seconds

    // Run in reverse
    analogWrite(RPWM, 0);
    analogWrite(LPWM, maxSpeed); // Use max speed
    delay(2000); // Run for 2 seconds
    
    // Stop motor
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    delay(500); // Pause for 0.5 seconds
}
