// Laser.ino - points a laser at the place the balance weight goes.
//
// The servo only turns 180 deg, so two lasers sit back to back on it. Angles
// 0..180 use laser 1, 180..360 use laser 2 with the servo turned 180 less.
//
// The servo pulse is made entirely by Timer2 interrupts (no Servo.h, which
// would take Timer1 and kill PWM on pins 9/10). Timer2 ticks every 4 us and
// wraps every 1 ms; a 20 ms frame is 20 wraps:
//   wrap 0            -> pin high
//   wrap 'wholeMs'    -> arm compare B for the remaining 'fracTicks'
//   compare B         -> pin low
// The pulse width therefore does not depend on what loop() is doing.

static volatile uint8_t servoFrameMs   = 0;      // 0..19, position inside the frame
static volatile uint8_t servoWholeMs   = 1;      // pulse width: whole milliseconds
static volatile uint8_t servoFracTicks = 125;    // pulse width: remainder in 4 us ticks
static volatile bool    servoEnabled   = false;

ISR(TIMER2_COMPA_vect) {
  uint8_t ms = servoFrameMs + 1;
  if (ms >= 20) ms = 0;
  servoFrameMs = ms;
  if (!servoEnabled) return;

  if (ms == 0) SERVO_PORT |= _BV(SERVO_BIT);
  if (ms == servoWholeMs) {
    if (servoFracTicks < 3) {
      SERVO_PORT &= ~_BV(SERVO_BIT);             // too close to the wrap to time: end now
    } else {
      OCR2B  = servoFracTicks;
      TIFR2  = _BV(OCF2B);                       // clear a stale match
      TIMSK2 |= _BV(OCIE2B);
    }
  }
}

ISR(TIMER2_COMPB_vect) {
  SERVO_PORT &= ~_BV(SERVO_BIT);
  TIMSK2 &= ~_BV(OCIE2B);
}

void laserInit() {
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_LASER1, OUTPUT);
  pinMode(PIN_LASER2, OUTPUT);
  digitalWrite(PIN_SERVO, LOW);
  digitalWrite(PIN_LASER1, LOW);
  digitalWrite(PIN_LASER2, LOW);

  TCCR2A = _BV(WGM21);          // CTC, top = OCR2A
  TCCR2B = _BV(CS22);           // prescaler 64 -> 4 us per tick
  OCR2A  = 249;                 // 250 ticks = 1 ms
  TCNT2  = 0;
  TIMSK2 = _BV(OCIE2A);
}

// True when an HX711 read (interrupts off for a moment) cannot disturb a pulse.
bool servoQuiet() {
  if (!servoEnabled) return true;
  uint8_t ms = servoFrameMs;
  return ms >= 4 && ms <= 17;
}

static void servoWrite(uint16_t us) {
  uint8_t whole = us / 1000;
  uint8_t frac  = (us % 1000) / 4;
  noInterrupts();
  servoWholeMs   = whole;
  servoFracTicks = frac;
  servoEnabled   = true;
  interrupts();
}

static void servoOff() {
  noInterrupts();
  servoEnabled = false;
  TIMSK2 &= ~_BV(OCIE2B);
  SERVO_PORT &= ~_BV(SERVO_BIT);
  interrupts();
}

// Call after anything that changes the laser: on/off, manual mode, a new angle,
// LaserOffset or a new balance result.
void laserApply() {
  if (!laserOn && !manualLaser) {
    digitalWrite(PIN_LASER1, LOW);
    digitalWrite(PIN_LASER2, LOW);
    servoOff();                  // no pulses: the servo relaxes
    return;
  }

  float a = fmod((manualLaser ? manualAngle : resAngle) + cfg.laserOffset, 360.0);
  if (a < 0) a += 360.0;

  bool first = (a <= 180.0);
  if (!first) a -= 180.0;
  servoWrite(SERVO_US_0 + (uint16_t)((SERVO_US_180 - SERVO_US_0) * (a / 180.0)));
  digitalWrite(PIN_LASER1, first ? HIGH : LOW);
  digitalWrite(PIN_LASER2, first ? LOW : HIGH);
}
