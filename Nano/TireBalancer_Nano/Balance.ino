// Balance.ino - load cells, tare, calibration and the balance calculation.
//
// The tire lies flat on three arms 120 deg apart, one load cell under each.
// With the platform level, the three readings give the total weight and the
// position of the centre of gravity. The balance weight goes on the opposite
// side, sized so its moment cancels the offset.
//
// The HX711s are only read while something needs them (a Tare / Balance / Cal
// job, the scale debug stream, or Mass Comp during leveling).

static HX711   hx[3];
static int32_t raw[3]      = {0, 0, 0};
static bool    fresh[3]    = {false, false, false};
static float   tareRaw[3]  = {0, 0, 0};
static bool    tared       = false;

static uint8_t  job        = JOB_NONE;
static int32_t  jobSum[3];
static uint8_t  jobCount;
static uint32_t jobSetStart;
static float    calMassKg;

void scaleInit() {
  for (uint8_t i = 0; i < 3; i++) hx[i].begin(PIN_HX_DOUT[i], PIN_HX_SCK[i]);
}

static void updateWeights() {
  float extra = testMassOn ? cfg.testMass : 0;
  for (uint8_t i = 0; i < 3; i++) weight[i] = (raw[i] - tareRaw[i]) / cfg.cal[i] + extra;
}

// ------------------------------------------------------------ jobs
// A job averages JOB_SAMPLES complete sets of readings, then acts on the result.
void scaleJobStart(uint8_t newJob, float massKg) {
  if (state == ST_SAMPLING && job == newJob) return;     // already at it
  if (state != ST_IDLE) { eventError(ERR_BUSY); return; }
  // The zero point is not stored: Tare (empty arms) once after every power-up.
  if (newJob != JOB_TARE && !tared) { eventError(ERR_NO_TARE); return; }
  for (uint8_t i = 0; i < 3; i++) {
    if (hx[i].is_ready()) hx[i].read();      // throw away a reading that has been waiting
    fresh[i]  = false;
    jobSum[i] = 0;
  }
  job         = newJob;
  jobCount    = 0;
  calMassKg   = massKg;
  jobSetStart = millis();
  state       = ST_SAMPLING;
}

// Testing only: forget the zero point and use the raw readings as they are.
void scaleTareClear() {
  for (uint8_t i = 0; i < 3; i++) tareRaw[i] = 0;
  tared = true;
  updateWeights();
}

void scaleJobCancel() {
  job = JOB_NONE;
  if (state == ST_SAMPLING) state = ST_IDLE;
}

static void jobFinish() {
  uint8_t done = job;
  float avg[3];
  for (uint8_t i = 0; i < 3; i++) avg[i] = (float)jobSum[i] / jobCount;
  job   = JOB_NONE;
  state = ST_IDLE;

  if (done == JOB_TARE) {
    for (uint8_t i = 0; i < 3; i++) tareRaw[i] = avg[i];
    tared = true;
    updateWeights();
    eventRaise(EV_TARE_DONE, 0);
    return;
  }

  if (done >= JOB_CAL_A) {
    // Known mass sitting on ONE arm: counts per kg for that load cell.
    uint8_t i = done - JOB_CAL_A;
    float counts = avg[i] - tareRaw[i];
    if (fabs(counts) < MIN_CAL_COUNTS || calMassKg <= 0) { eventError(ERR_CAL); return; }
    cfg.cal[i] = counts / calMassKg;
    settingsSave();
    Serial.print(F("# Cal"));
    Serial.print((char)('A' + i));
    Serial.print(' ');
    Serial.println(cfg.cal[i], 1);
    return;
  }

  // JOB_BALANCE
  float w[3];
  float extra = testMassOn ? cfg.testMass : 0;
  for (uint8_t i = 0; i < 3; i++) w[i] = (avg[i] - tareRaw[i]) / cfg.cal[i] + extra;

  if (rimD <= 0 || treadW <= 0 || sideWallPct <= 0) { eventError(ERR_TIRE_DATA); return; }
  if (!balanceCompute(w, resAngle, resMassG, resTotalKg)) { eventError(ERR_NO_LOAD); return; }

  laserOn = true;              // point at the spot straight away
  laserApply();
  eventRaise(EV_BALANCE_DONE, 0);
}

// ------------------------------------------------------------ polling
void scalePoll(uint32_t now) {
  // An HX711 read switches interrupts off for a moment, which would stretch a
  // servo pulse. Only read in the quiet part of the 20 ms servo frame.
  if (!servoQuiet()) return;

  for (uint8_t i = 0; i < 3; i++) {
    if (hx[i].is_ready()) {
      raw[i]   = (int32_t)hx[i].read();
      fresh[i] = true;
      if (hxReads[i] < 255) hxReads[i]++;
    }
  }

  if (fresh[0] && fresh[1] && fresh[2]) {
    fresh[0] = fresh[1] = fresh[2] = false;
    updateWeights();
    if (job != JOB_NONE) {
      for (uint8_t i = 0; i < 3; i++) jobSum[i] += raw[i];
      jobSetStart = now;
      if (++jobCount >= JOB_SAMPLES) jobFinish();
    }
  } else if (job != JOB_NONE && (int32_t)(now - jobSetStart) > (int32_t)SCALE_TIMEOUT_MS) {
    job   = JOB_NONE;
    state = ST_IDLE;
    eventError(ERR_SCALE);
  }
}

// ------------------------------------------------------------ the maths
// w[] = kg on load cells A (210 deg), B (90 deg), C (330 deg).
// Returns false when there is no usable load or no tire data.
//
//   contactR   = rim radius + half the sidewall height   (where the sidewall
//                bulge rests on the arms)
//   CG offset  = contactR * |sum(w_i * unit_i)| / total
//   weight     = total * CG offset / weightR             (moment balance)
//   angle      = direction of the CG + 180 deg
bool balanceCompute(const float w[3], float &angleDeg, float &massG, float &totalKg) {
  totalKg = w[0] + w[1] + w[2];
  float x = w[0] * LC_A_COS + w[1] * LC_B_COS + w[2] * LC_C_COS;
  float y = w[0] * LC_A_SIN + w[1] * LC_B_SIN + w[2] * LC_C_SIN;

  angleDeg = atan2(y, x) * RAD_TO_DEG + 180.0f;       // opposite the heavy side, 0..360
  if (angleDeg >= 360.0f) angleDeg -= 360.0f;
  massG = 0;

  if (totalKg < MIN_TOTAL_KG || rimD <= 0) return false;

  float sideWallH = treadW * sideWallPct * 0.01f;
  float contactR  = rimD * 0.5f + sideWallH * 0.5f;
  float placeR    = (weightR > 0) ? weightR : rimD * 0.5f;
  massG = contactR * sqrt(x * x + y * y) / placeR * 1000.0f;
  return true;
}
