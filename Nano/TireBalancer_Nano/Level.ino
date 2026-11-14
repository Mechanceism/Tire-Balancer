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
static float    gyroBiasX    = 0;      // gyro zero, raw counts, machine X / Y
static float    gyroBiasY    = 0;

static void toMachine(const int8_t ax[2], const int16_t s[3], int16_t m[3]);
static uint8_t  spikeRun     = 0;
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

  // Starting gyro zero (the platform is still at power-up); tracked from then on.
  long sx = 0, sy = 0;
  for (uint8_t i = 0; i < 32; i++) {
    int16_t s[3], m[3];
    mpuTop.getRotation(&s[0], &s[1], &s[2]);
    toMachine(cfg.topAxis, s, m);
    sx += m[0];
    sy += m[1];
    delay(3);
  }
  gyroBiasX = sx / 32.0f;
  gyroBiasY = sy / 32.0f;
}

// ------------------------------------------------------------ MPU orientation
// Machine axes: +X toward 0 deg (right), +Y toward 90 deg (far side), +Z up.
// cfg.topAxis / cfg.baseAxis = {sensor axis along machine +X, along machine +Y}:
// 1 = sensor X, 2 = sensor Y, 3 = sensor Z, negative = pointing the other way.
// Machine Z follows from those two (right-handed), so an upside-down MPU needs
// no extra setting.
bool axesValid(const int8_t ax[2]) {
  uint8_t a = abs(ax[0]), b = abs(ax[1]);
  return a >= 1 && a <= 3 && b >= 1 && b <= 3 && a != b;
}

// Sensor axis code along machine +Z = (axis along X) cross (axis along Y)
static int8_t axisZ(int8_t x, int8_t y) {
  int8_t a = abs(x), b = abs(y);
  int8_t s = ((a % 3) + 1 == b) ? 1 : -1;     // X*Y=Z, Y*Z=X, Z*X=Y are positive
  if (x < 0) s = -s;
  if (y < 0) s = -s;
  return s * (6 - a - b);
}

static int16_t pickAxis(int8_t code, const int16_t s[3]) {
  int16_t v = s[abs(code) - 1];
  if (code > 0) return v;
  return (v == -32768) ? 32767 : -v;
}

// Sensor reading (x, y, z) -> machine (X, Y, Z). Works for accel and gyro.
static void toMachine(const int8_t ax[2], const int16_t s[3], int16_t m[3]) {
  m[0] = pickAxis(ax[0], s);
  m[1] = pickAxis(ax[1], s);
  m[2] = pickAxis(axisZ(ax[0], ax[1]), s);
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

// ------------------------------------------------------------ reach area
// Is level (the target) inside the area the platform can reach on this frame?
// Positive = inside by that many degrees, negative = outside.
// Falls back to the plain frame-tilt limit if no area was ever measured.
static float reachMarginFor(float x, float y) {
  float area = 0;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t j = (i + 1) & 3;
    area += cfg.reach[i][0] * cfg.reach[j][1] - cfg.reach[j][0] * cfg.reach[i][1];
  }
  if (fabs(area) < 2.0f) return MAX_BASE_ANGLE - baseAngle;   // never measured
  float s = (area > 0) ? 1.0f : -1.0f;              // works for either corner order
  float m = 1e6;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t j = (i + 1) & 3;
    float ex = cfg.reach[j][0] - cfg.reach[i][0];
    float ey = cfg.reach[j][1] - cfg.reach[i][1];
    float len = sqrt(ex * ex + ey * ey);
    if (len < 0.01f) continue;
    float d = s * (ex * (y - cfg.reach[i][1]) - ey * (x - cfg.reach[i][0])) / len;
    if (d < m) m = d;
  }
  return m;
}

static void readBase() {
  int16_t s[3], m[3];
  mpuBottom.getAcceleration(&s[0], &s[1], &s[2]);
  toMachine(cfg.baseAxis, s, m);
  baseAz    = m[2];
  basePitch = tiltDeg(m[1]);      // 90 deg side up -> +
  baseRoll  = tiltDeg(m[0]);      // 0 deg side up -> +
  baseAngle = sqrt(basePitch * basePitch + baseRoll * baseRoll);

  // Where level is, in corner angles relative to the frame
  float tp = cfg.pitchOffset + pitchTrim, tr = cfg.rollOffset + rollTrim;
  float x  = (tp - MIX_ROLL * tr) - (basePitch - MIX_ROLL * baseRoll);
  float y  = (tp + MIX_ROLL * tr) - (basePitch + MIX_ROLL * baseRoll);
  reachMargin = reachMarginFor(x, y);
}

// ------------------------------------------------------------ start / stop
void levelStart() {
  if (state == ST_LEVELING) return;                 // already at it
  if (state != ST_IDLE)             { eventError(ERR_BUSY);       return; }
  if (!mpuPresent(MPU_TOP_ADDR))    { eventError(ERR_MPU_TOP);    return; }
  if (!mpuPresent(MPU_BOTTOM_ADDR)) { eventError(ERR_MPU_BOTTOM); return; }
  if (!axesValid(cfg.topAxis) || !axesValid(cfg.baseAxis)) { eventError(ERR_AXES); return; }
  readBase();
  if (reachMargin < REACH_MARGIN_START) { eventError(ERR_BASE_TILT); return; }

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

// ------------------------------------------------------------ FindReach
// Drives the platform to the four actuator end-stop combinations, records the
// platform-to-frame corner angles at each, saves them and then levels.
// An actuator counts as at its end stop when its corner angle moves less than
// REACH_STALL_DEG in 1 s (works with or without built-in limit switches).
static const int8_t REACH_DIR[4][2] = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
static uint8_t  reachLeg;
static uint32_t reachLegStart, reachCheckAt;
static float    reachRef[2];
static bool     reachEnd[2];

static void reachLegBegin(uint32_t now) {
  reachLegStart = now;
  reachCheckAt  = now + 1800;
  reachEnd[0] = reachEnd[1] = false;
  reachRef[0] = pitchF - MIX_ROLL * rollF;
  reachRef[1] = pitchF + MIX_ROLL * rollF;
}

void reachStart() {
  if (state != ST_IDLE) { eventError(ERR_BUSY); return; }
  if (!mpuPresent(MPU_TOP_ADDR) || !mpuPresent(MPU_BOTTOM_ADDR)) { eventError(ERR_REACH); return; }
  if (!axesValid(cfg.topAxis) || !axesValid(cfg.baseAxis)) { eventError(ERR_AXES); return; }
  actuatorsStop();
  reachLeg = 0;
  reachLegBegin(millis());
  state = ST_REACH;
}

static void reachTick(uint32_t now) {
  float c[2] = {pitchF - MIX_ROLL * rollF, pitchF + MIX_ROLL * rollF};
  for (uint8_t i = 0; i < 2; i++) actuatorDrive(i, reachEnd[i] ? 0 : REACH_DIR[reachLeg][i] * REACH_PWM);

  if ((int32_t)(now - reachCheckAt) >= 0) {
    for (uint8_t i = 0; i < 2; i++) {
      if (!reachEnd[i] && fabs(c[i] - reachRef[i]) < REACH_STALL_DEG) reachEnd[i] = true;
      reachRef[i] = c[i];
    }
    reachCheckAt = now + 1000;
  }

  if (reachEnd[0] && reachEnd[1]) {
    readBase();
    cfg.reach[reachLeg][0] = c[0] - (basePitch - MIX_ROLL * baseRoll);
    cfg.reach[reachLeg][1] = c[1] - (basePitch + MIX_ROLL * baseRoll);
    Serial.print(F("# Reach "));
    Serial.print(cfg.reach[reachLeg][0], 2);
    Serial.print(',');
    Serial.println(cfg.reach[reachLeg][1], 2);
    if (++reachLeg >= 4) {
      actuatorsStop();
      state = ST_IDLE;
      settingsSave();
      levelStart();               // back to level (raises LevelCompleted as usual)
      return;
    }
    reachLegBegin(now);
  } else if ((int32_t)(now - reachLegStart) > (int32_t)REACH_LEG_MS) {
    levelAbort(ERR_REACH);
  }
}

// ------------------------------------------------------------ jog (commissioning)
// JogLeft<pwm> / JogRight<pwm>: run one actuator at a signed PWM for JOG_MS, then stop.
// Used to find each actuator's direction and the lowest PWM that moves it.
// Both actuators can jog at once (each has its own end time). JogTime<ms>
// changes the length (not saved), e.g. to drive to an end stop.
static int16_t  jogCmd[2] = {0, 0};
static uint32_t jogEnd[2];
static uint16_t jogMs = JOG_MS;

void jogSetTime(float ms) {
  jogMs = (uint16_t)constrain(ms, 100, 40000);
}

void jogStart(uint8_t i, float pwm) {
  if (state != ST_IDLE && state != ST_JOG) { eventError(ERR_BUSY); return; }
  if (state == ST_IDLE) {
    actuatorsStop();
    jogCmd[0] = jogCmd[1] = 0;
  }
  jogCmd[i] = (int16_t)constrain(pwm, -255, 255);
  jogEnd[i] = millis() + jogMs;
  state = ST_JOG;
}

void jogTick(uint32_t now) {
  if (state != ST_JOG) return;
  bool any = false;
  for (uint8_t i = 0; i < 2; i++) {
    if (jogCmd[i] != 0 && (int32_t)(now - jogEnd[i]) >= 0) jogCmd[i] = 0;
    actuatorDrive(i, jogCmd[i]);
    if (jogCmd[i] != 0) any = true;
  }
  if (!any) {
    actuatorsStop();
    state = ST_IDLE;
  }
}

// ------------------------------------------------------------ PID
// Returns the PID output for actuator i (0 while it is resting inside the band).
// e is the corner error in degrees. The caller turns this into PWM.
static float pidStep(uint8_t i, float e, float dt, float gain) {
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

  if (fabs(u) < cfg.maxSpeed - cfg.minSpeed) a.integ += e * dt;   // anti-windup: no integrating while saturated
  if (u == 0) u = (e > 0) ? 0.001f : -0.001f;                      // active means moving, even at minSpeed
  return u;
}

// The actuators do not move below minSpeed, so the PID output rides on top of
// it. If either output is past the limit both are scaled down together, so the
// difference between them (the roll correction) is kept.
static void driveBoth(float u1, float u2) {
  float span = cfg.maxSpeed - cfg.minSpeed;
  float m = max(fabs(u1), fabs(u2));
  if (m > span && m > 0) {
    u1 *= span / m;
    u2 *= span / m;
  }
  float u[2] = {u1, u2};
  for (uint8_t i = 0; i < 2; i++) {
    if (u[i] == 0) { actuatorDrive(i, 0); continue; }
    int16_t speed = (int16_t)(cfg.minSpeed + fabs(u[i]));
    actuatorDrive(i, u[i] > 0 ? speed : -speed);
  }
}

// ------------------------------------------------------------ control tick
// Runs every LEVEL_PERIOD_MS while leveling (or while the level debug stream is on).
void levelTick(uint32_t now, float dt) {
  int16_t s[6], a[3], g[3];
  mpuTop.getMotion6(&s[0], &s[1], &s[2], &s[3], &s[4], &s[5]);
  toMachine(cfg.topAxis, s, a);          // accel, machine axes
  toMachine(cfg.topAxis, s + 3, g);      // gyro, machine axes
  int16_t ax = a[0], ay = a[1], az = a[2];
  gyroX = g[0];
  gyroY = g[1];
  topAz = az;

  // Gyro zero drifts (it was -4 deg/s on one axis when first measured):
  // follow it whenever both actuators have been off for a moment.
  uint32_t ms = millis();
  if (act[0].dir == 0 && act[1].dir == 0 &&
      ms - act[0].restAt > GYRO_REST_MS && ms - act[1].restAt > GYRO_REST_MS) {
    gyroBiasX += GYRO_BIAS_RATE * (g[0] - gyroBiasX);
    gyroBiasY += GYRO_BIAS_RATE * (g[1] - gyroBiasY);
  }
  // Rotation about machine +X raises the 90 deg side (pitch up); rotation
  // about machine +Y lowers the 0 deg side (roll down).
  float rateP =  (g[0] - gyroBiasX) / GYRO_COUNTS_DPS;   // deg/s
  float rateR = -(g[1] - gyroBiasY) / GYRO_COUNTS_DPS;
  bool  useGyro = cfg.gyroMix > 0;

  // A live accelerometer always has a little noise. Identical readings for
  // half a second mean the sensor or the I2C bus has stopped answering.
  if (ax == lastAx && ay == lastAy && az == lastAz) {
    if (staleTicks < 255) staleTicks++;
  } else {
    staleTicks = 0;
  }
  lastAx = ax; lastAy = ay; lastAz = az;

  float pitch = tiltDeg(ay);             // 90 deg side up -> +
  float roll  = tiltDeg(ax);             // 0 deg side up -> +
  pitchRaw = pitch;
  rollRaw  = roll;

  // Skip samples that are not tilt. The accelerometer measures gravity PLUS the
  // platform's own acceleration: smooth actuator motion only adds ~0.02 g, but
  // a jolt (slack taken up on a hard reversal) can add a lot for a moment.
  // Skipped: an I2C read that timed out, a total far from 1 g (outside about
  // 0.71-1.22 g), or a jump bigger than the platform can tilt in one tick.
  // Without the gyro the last good angle is held; with it the gyro carries the
  // angle through. Five in a row (0.1 s) is accepted as real.
  bool bad = Wire.getWireTimeoutFlag();
  Wire.clearWireTimeoutFlag();
  float gSq = ((float)ax * ax + (float)ay * ay + (float)az * az) / (ACCEL_COUNTS_1G * ACCEL_COUNTS_1G);
  if (gSq < 0.5f || gSq > 1.5f) bad = true;
  if (filterSeeded && (fabs(pitch - pitchF) > SPIKE_DEG || fabs(roll - rollF) > SPIKE_DEG)) bad = true;
  bool accelOk = true;
  if (bad) {
    if (rejectedSamples < 65535) rejectedSamples++;
    if (++spikeRun < SPIKE_TICKS) {
      if (!useGyro || !filterSeeded) return;   // hold the last angle (and outputs) for this tick
      accelOk = false;                         // gyro only this tick
    } else {
      filterSeeded = false;                    // persistent: accept it as the new position
      spikeRun = 0;
    }
  } else {
    spikeRun = 0;
  }

  if (!filterSeeded) {
    pitchF = pitch;
    rollF  = roll;
    filterSeeded = true;
  } else if (useGyro) {
    // Complementary filter: the gyro follows fast motion with no lag, the
    // accelerometer slowly pulls the result back to true gravity.
    pitchF += rateP * dt;
    rollF  += rateR * dt;
    if (accelOk) {
      pitchF = cfg.gyroMix * pitchF + (1 - cfg.gyroMix) * pitch;
      rollF  = cfg.gyroMix * rollF  + (1 - cfg.gyroMix) * roll;
    }
  } else {
    pitchF = cfg.alpha * pitchF + (1 - cfg.alpha) * pitch;
    rollF  = cfg.alpha * rollF  + (1 - cfg.alpha) * roll;
  }

  if (++baseTicks >= BASE_CHECK_TICKS) {
    baseTicks = 0;
    readBase();
  }

  if (state == ST_REACH) { reachTick(now); return; }
  if (state != ST_LEVELING) return;    // debug stream only: measure, do not move

  if (staleTicks >= STALE_TICKS)        { levelAbort(ERR_MPU_TOP);   return; }
  if (reachMargin < REACH_MARGIN_RUN)   { levelAbort(ERR_BASE_TILT); return; }   // frame moved: level out of reach
  if (autoStop && (int32_t)(now - levelStartMs) > (int32_t)LEVEL_TIMEOUT_MS) { levelAbort(ERR_LEVEL_TIMEOUT); return; }

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

  driveBoth(pidStep(0, e1, dt, g1), pidStep(1, e2, dt, g2));

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
