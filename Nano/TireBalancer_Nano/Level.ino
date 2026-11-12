// Level.ino - platform leveling.
//
// The platform MPU gives pitch and roll relative to gravity. Each actuator has
// its own PID loop working on the height error at its corner. The frame MPU is
// only a guard: if the frame itself is tilted further than the actuators can
// correct there is no point trying.

MPU6050 mpuTop(MPU_TOP_ADDR);
MPU6050 mpuBottom(MPU_BOTTOM_ADDR);

struct Actuator {
  uint8_t  pinUp, pinDown;
  int8_t   dir;        // what the pins are doing right now: -1 down, 0 off, +1 up
  uint32_t restAt;     // millis() when it was last switched off
  bool     active;     // the PID is currently trying to move this actuator
  uint8_t  armTicks;   // ticks in a row the error has been outside the band
  float    integ;      // integral of the error, deg*s
  float    prevErr;    // error on the previous tick, deg
  int16_t  out;        // last command, signed PWM (for the debug stream)
};

static Actuator act[2] = {
  {PIN_ACT1_UP, PIN_ACT1_DOWN, 0, 0, false, 0, 0, 0, 0},
  {PIN_ACT2_UP, PIN_ACT2_DOWN, 0, 0, false, 0, 0, 0, 0},
};

static bool     filterSeeded = false;
static uint8_t  stableTicks  = 0;
static uint8_t  staleTicks   = 0;
static uint8_t  baseTicks    = 0;
static uint32_t levelStartMs = 0;
static int16_t  lastAx, lastAy, lastAz;

// ------------------------------------------------------------ actuators
void actuatorsInit() {
  for (uint8_t i = 0; i < 2; i++) {
    digitalWrite(act[i].pinUp, LOW);     // low first, then output: no glitch
    digitalWrite(act[i].pinDown, LOW);
    pinMode(act[i].pinUp, OUTPUT);
    pinMode(act[i].pinDown, OUTPUT);
  }
}

// The only function that writes the H-bridge pins.
// cmd is signed PWM: > 0 up, < 0 down, 0 off.
// Every change of direction (including starting from rest) first switches both
// inputs off and waits DEAD_TIME_MS, so UP and DOWN can never overlap.
void actuatorDrive(uint8_t i, int16_t cmd) {
  Actuator &a = act[i];
  int8_t  want = (cmd > 0) - (cmd < 0);
  uint8_t pwm  = (uint8_t)constrain(abs(cmd), 0, 255);
  a.out = cmd;

  if (want != a.dir) {
    analogWrite(a.pinUp, 0);
    analogWrite(a.pinDown, 0);
    if (a.dir != 0) a.restAt = millis();
    a.dir = 0;
    if (want == 0 || millis() - a.restAt < DEAD_TIME_MS) return;
    a.dir = want;
  }
  if (a.dir > 0)      analogWrite(a.pinUp, pwm);
  else if (a.dir < 0) analogWrite(a.pinDown, pwm);
}

void actuatorsStop() {
  for (uint8_t i = 0; i < 2; i++) {
    actuatorDrive(i, 0);
    act[i].active   = false;
    act[i].armTicks = 0;
    act[i].integ    = 0;
  }
}

int16_t actuatorOutput(uint8_t i) {
  return act[i].out;
}

// ------------------------------------------------------------ sensors
void levelInit() {
  Wire.begin();
  Wire.setWireTimeout(25000, true);    // a stuck I2C bus must never hang the loop

  mpuTop.initialize();
  mpuBottom.initialize();
  mpuTop.setDLPFMode(MPU6050_DLPF_BW_5);      // 5 Hz low-pass inside the sensor
  mpuBottom.setDLPFMode(MPU6050_DLPF_BW_5);

  // Frame MPU calibration (from Tools/MPU6050_Calibrate)
  mpuBottom.setXAccelOffset(-414);
  mpuBottom.setYAccelOffset(429);
  mpuBottom.setZAccelOffset(4791);
}

static bool mpuPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Tilt from one accelerometer axis. Uses 1 g as the reference instead of the Z
// reading so a mis-calibrated Z axis cannot scale the angle.
static float tiltDeg(int16_t counts) {
  return atan(counts / ACCEL_COUNTS_1G) * RAD_TO_DEG;
}

static void readBase() {
  int16_t ax, ay, az;
  mpuBottom.getAcceleration(&ax, &ay, &az);
  float p = tiltDeg(ax), r = tiltDeg(ay);
  baseAngle = sqrt(p * p + r * r);
}

// ------------------------------------------------------------ start / stop
void levelStart() {
  if (state == ST_LEVELING) return;                 // already at it
  if (state != ST_IDLE)             { eventError(ERR_BUSY);       return; }
  if (!mpuPresent(MPU_TOP_ADDR))    { eventError(ERR_MPU_TOP);    return; }
  if (!mpuPresent(MPU_BOTTOM_ADDR)) { eventError(ERR_MPU_BOTTOM); return; }
  readBase();
  if (baseAngle >= MAX_BASE_ANGLE)  { eventError(ERR_BASE_TILT);  return; }

  actuatorsStop();
  filterSeeded = false;
  stableTicks  = 0;
  staleTicks   = 0;
  baseTicks    = 0;
  levelStartMs = millis();
  state = ST_LEVELING;
}

void levelCancel() {
  if (state == ST_LEVELING) {
    actuatorsStop();
    state = ST_IDLE;
  }
}

static void levelAbort(uint8_t err) {
  actuatorsStop();
  state = ST_IDLE;
  eventError(err);
}

// ------------------------------------------------------------ PID
// Returns signed PWM for actuator i. e is the corner error in degrees.
static int16_t pidStep(uint8_t i, float e, float dt, float gain) {
  Actuator &a = act[i];
  float mag  = fabs(e);
  float band = cfg.levelTol * 0.5f;      // start moving outside this...
  if (!a.active) {
    if (mag <= band) { a.armTicks = 0; return 0; }
    if (++a.armTicks < ARM_TICKS) return 0;    // ...for a few ticks in a row (not just noise)
    a.active  = true;
    a.integ   = 0;
    a.prevErr = e;
  } else if (mag < band * 0.5f) {        // ...and stop once well inside it
    a.active   = false;
    a.armTicks = 0;
    a.integ    = 0;
    return 0;
  }

  if ((e > 0) != (a.integ > 0)) a.integ = 0;   // crossed zero: drop the old integral

  float d = (e - a.prevErr) / dt;
  a.prevErr = e;
  float u = (cfg.kp * e + cfg.ki * a.integ + cfg.kd * d) * gain;

  // The actuators do not move below minSpeed, so the PID output rides on top of it.
  float span = cfg.maxSpeed - cfg.minSpeed;
  if (fabs(u) < span) a.integ += e * dt;       // anti-windup: no integrating while saturated
  float speed = constrain(cfg.minSpeed + fabs(u), cfg.minSpeed, cfg.maxSpeed);
  return (u >= 0) ? (int16_t)speed : -(int16_t)speed;
}

// ------------------------------------------------------------ control tick
// Runs every LEVEL_PERIOD_MS while leveling (or while the level debug stream is on).
void levelTick(uint32_t now, float dt) {
  int16_t ax, ay, az;
  mpuTop.getAcceleration(&ax, &ay, &az);

  // A live accelerometer always has a little noise. Identical readings for
  // half a second mean the sensor or the I2C bus has stopped answering.
  if (ax == lastAx && ay == lastAy && az == lastAz) {
    if (staleTicks < 255) staleTicks++;
  } else {
    staleTicks = 0;
  }
  lastAx = ax; lastAy = ay; lastAz = az;

  float pitch = tiltDeg(ax);
  float roll  = tiltDeg(ay);
  if (!filterSeeded) {
    pitchF = pitch;
    rollF  = roll;
    filterSeeded = true;
  } else {
    pitchF = cfg.alpha * pitchF + (1 - cfg.alpha) * pitch;
    rollF  = cfg.alpha * rollF  + (1 - cfg.alpha) * roll;
  }

  if (++baseTicks >= BASE_CHECK_TICKS) {
    baseTicks = 0;
    readBase();
  }

  if (state != ST_LEVELING) return;    // debug stream only: measure, do not move

  if (staleTicks >= STALE_TICKS)   { levelAbort(ERR_MPU_TOP);   return; }
  if (baseAngle >= MAX_BASE_ANGLE) { levelAbort(ERR_BASE_TILT); return; }
  if (autoStop && now - levelStartMs > LEVEL_TIMEOUT_MS) { levelAbort(ERR_LEVEL_TIMEOUT); return; }

  float ePitch = cfg.pitchOffset + pitchTrim - pitchF;
  float eRoll  = cfg.rollOffset  + rollTrim  - rollF;

  float e1 = ePitch - MIX_ROLL * eRoll;      // actuator 1 at 150 deg
  float e2 = ePitch + MIX_ROLL * eRoll;      // actuator 2 at 30 deg

  // Mass compensation: a loaded platform needs more drive for the same speed.
  // Actuator 1 sits between load cells A and B, actuator 2 between B and C.
  float g1 = 1, g2 = 1;
  if (massComp) {
    g1 = 1 + constrain(sqrt(weight[0] * weight[0] + weight[1] * weight[1]) / MASS_COMP_FULL_KG, 0, 1);
    g2 = 1 + constrain(sqrt(weight[1] * weight[1] + weight[2] * weight[2]) / MASS_COMP_FULL_KG, 0, 1);
  }

  actuatorDrive(0, pidStep(0, e1, dt, g1));
  actuatorDrive(1, pidStep(1, e2, dt, g2));

  // Both actuators resting inside their bands means pitch and roll are both
  // inside levelTol (see MIX_ROLL: |pitch| < tol/2, |roll| < 0.87 tol).
  if (!act[0].active && !act[1].active) {
    if (stableTicks < 255) stableTicks++;
  } else {
    stableTicks = 0;
  }

  if (autoStop && stableTicks >= STABLE_TICKS) {
    actuatorsStop();
    state = ST_IDLE;
    eventRaise(EV_LEVEL_DONE, 0);
  }
}
