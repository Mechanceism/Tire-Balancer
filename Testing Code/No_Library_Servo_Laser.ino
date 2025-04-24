const int servoPin = 4;
int Laser1 = A2;
int Laser2 = A3;

int degree0 = 2260;
int degree180 = 550;

volatile int pulseWidth = degree0;  // In microseconds
volatile bool doPulse = false;

void setup() {
  pinMode(servoPin, OUTPUT);
  Serial.begin(9600);
  setupServoTimer2();

  pinMode(Laser1, OUTPUT);
  pinMode(Laser2, OUTPUT);
  digitalWrite(Laser1, LOW);
  digitalWrite(Laser2, LOW);
}

void loop() {
  // Handle serial input
  if (Serial.available()) {
    int input = Serial.parseInt();
    if (input >= 1 && input <= 360) {
      noInterrupts();

      if (input <= 180) {
      pulseWidth = map(input, 0,180,degree0,degree180);
      digitalWrite(Laser1, HIGH);
      digitalWrite(Laser2, LOW);
    } else {
      pulseWidth = map(input-180, 0,180,degree0,degree180);
      digitalWrite(Laser1, LOW);
      digitalWrite(Laser2, HIGH);
    }

      //int degree = map(input, 0,180,degree0,degree180);
      //pulseWidth = degree;
      interrupts();
      Serial.print("Pulse width set to: ");
      Serial.println(pulseWidth);
    }
  }

  
}

// Timer2 triggers every 20ms
void setupServoTimer2() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 249;             // 1ms tick (16MHz / 64 / 250)
  TCCR2A |= (1 << WGM21);  // CTC mode
  TCCR2B |= (1 << CS22);   // Prescaler 64
  TIMSK2 |= (1 << OCIE2A); // Enable compare match interrupt
}

ISR(TIMER2_COMPA_vect) {
  static uint8_t msCount = 0;
  msCount++;

  if (msCount >= 20) {
    msCount = 0;
    doPulse = true;  // Trigger the pulse in main loop
  }
}
