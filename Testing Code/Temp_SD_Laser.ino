#include <Servo.h>

Servo laserServo;

int pos = 0;
int Laser1 = A2;
int Laser2 = A3;

void setup() {
  laserServo.attach(4);  // attaches the servo on pin 9 to the servo object
  pinMode(Laser1, OUTPUT);
  pinMode(Laser2, OUTPUT);

  digitalWrite(Laser1, LOW);
  digitalWrite(Laser2, LOW);
}

void loop() {
  for (pos = 0; pos <= 360; pos += 1) { // goes from 0 degrees to 180 degrees
    if (pos <= 180) {
      laserServo.write(pos);
      digitalWrite(Laser1, HIGH);
      digitalWrite(Laser2, LOW);
    } else {
      laserServo.write(pos - 180);
      digitalWrite(Laser1, LOW);
      digitalWrite(Laser2, HIGH);
    }
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 360; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    if (pos <= 180) {
      laserServo.write(pos);
      digitalWrite(Laser1, HIGH);
      digitalWrite(Laser2, LOW);
    } else {
      laserServo.write(pos - 180);
      digitalWrite(Laser1, LOW);
      digitalWrite(Laser2, HIGH);
    }
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}


