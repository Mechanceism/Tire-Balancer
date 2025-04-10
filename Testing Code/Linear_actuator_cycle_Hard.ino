// Define motor driver pins
const int RPWM = 11;  // Forward PWM
const int LPWM = 10; // Reverse PWM
const int R_EN = 12;  // Forward enable
const int L_EN = 9;  // Reverse enable

void setup() {
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    
    digitalWrite(R_EN, HIGH); // Enable forward drive
    digitalWrite(L_EN, HIGH); // Enable reverse drive

    Serial.begin(9600); // Initialize serial communication
}

void loop() {
    // Run forward
    analogWrite(RPWM, 255); // Full speed forward
    analogWrite(LPWM, 0);
    delay(2000); // Run for 2 seconds
    
    // Stop motor
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    delay(500); // Pause for 0.5 seconds

    // Run in reverse
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 255); // Full speed reverse
    delay(2000); // Run for 2 seconds
    
    // Stop motor
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    delay(500); // Pause for 0.5 seconds
}
