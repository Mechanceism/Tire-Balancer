// Protocol.ino - serial link to the touch screen (and to you, in Serial Monitor).
//
// Commands are plain text, one or more per line, not case sensitive:
//     Level            a word on its own is an action
//     Kp35.5           a word followed by a number sets a value
//     MinSpeed60 MaxSpeed120
// The full list is in docs/Protocol.md and in runCommand() below.
//
// A line may end in "*XX" (two hex digits = XOR of every character before the
// '*'). The screen always adds it; if it is there it must be right or the line
// is dropped. Lines typed by hand without it are accepted.
//
// Events (LevelCompleted, BalanceCompleted ..., TareCompleted, Err ...) are sent
// with a checksum and repeated until the screen answers "Ack", so one garbled
// line cannot leave the screen waiting forever.
// Lines starting with '#' are for humans; the screen ignores them.

static char    lineBuf[LINE_MAX];
static uint8_t lineLen      = 0;
static bool    lineOverflow = false;   // line longer than LINE_MAX
static bool    lineNoise    = false;   // line contained non-text bytes

static uint8_t  evType  = EV_NONE;
static uint8_t  evArg   = 0;
static uint8_t  evTries = 0;
static uint32_t evNext  = 0;

// Prints to Serial while keeping a running XOR for the trailing checksum.
class ChkPrint : public Print {
 public:
  uint8_t x = 0;
  using Print::write;
  size_t write(uint8_t c) override {
    x ^= c;
    return Serial.write(c);
  }
  void end() {
    Serial.write('*');
    if (x < 16) Serial.write('0');
    Serial.print(x, HEX);
    Serial.println();
    x = 0;
  }
};
static ChkPrint out;

// ------------------------------------------------------------ events
void eventRaise(uint8_t type, uint8_t arg) {
  evType  = type;
  evArg   = arg;
  evTries = 0;
  evNext  = millis();
}

void eventError(uint8_t err) {
  eventRaise(EV_ERROR, err);
}

static void eventSend() {
  switch (evType) {
    case EV_LEVEL_DONE: out.print(F("LevelCompleted")); break;
    case EV_TARE_DONE:  out.print(F("TareCompleted"));  break;
    case EV_BALANCE_DONE:
      out.print(F("BalanceCompleted WeightAngle")); out.print(resAngle, 2);
      out.print(F(" WeightMass"));                  out.print(resMassG, 2);     // grams
      out.print(F(" TotalWeight"));                 out.print(resTotalKg, 2);   // kg
      break;
    case EV_ERROR:
      out.print(F("Err "));
      switch (evArg) {
        case ERR_BUSY:          out.print(F("Busy"));         break;
        case ERR_BASE_TILT:     out.print(F("BaseTilt"));     break;
        case ERR_MPU_TOP:       out.print(F("MPUTop"));       break;
        case ERR_MPU_BOTTOM:    out.print(F("MPUBottom"));    break;
        case ERR_LEVEL_TIMEOUT: out.print(F("LevelTimeout")); break;
        case ERR_SCALE:         out.print(F("Scale"));        break;
        case ERR_TIRE_DATA:     out.print(F("TireData"));     break;
        case ERR_NO_LOAD:       out.print(F("NoLoad"));       break;
        case ERR_NO_TARE:       out.print(F("NoTare"));       break;
        case ERR_CAL:           out.print(F("Cal"));          break;
      }
      break;
  }
  out.end();
}

void eventTick(uint32_t now) {
  if (evType == EV_NONE || (int32_t)(now - evNext) < 0) return;
  eventSend();
  if (++evTries >= EV_MAX_TRIES) evType = EV_NONE;
  else evNext = now + EV_RETRY_MS;
}

// ------------------------------------------------------------ replies
// Echo an accepted command as a '#' line, but only if it fits in the transmit
// buffer - a burst of settings from the screen must never stall loop().
static void confirm(const char *key, bool hasVal, float v) {
  if (Serial.availableForWrite() < 32) return;
  Serial.print(F("# "));
  Serial.print(key);
  if (hasVal) {
    Serial.print(' ');
    Serial.print(v);
  }
  Serial.println();
}

static void reject(const char *what) {
  if (Serial.availableForWrite() < 32) return;
  Serial.print(F("# ? "));
  Serial.println(what);
}

// ------------------------------------------------------------ commands
#define IS(name) (strcasecmp_P(key, PSTR(name)) == 0)

// Returns false for a word it does not know.
static bool runCommand(const char *key, bool hasVal, float v) {
  // ---- link
  if (IS("Ping"))            { Serial.println(F("Pong")); return true; }
  if (IS("Ack"))             { evType = EV_NONE; return true; }
  if (IS("Status"))          { if (state == ST_IDLE) settingsPrint(); else reject("busy"); return true; }

  // ---- actions
  if (IS("Stop"))            { stopAll(); }
  else if (IS("Level"))        { levelStart(); }
  else if (IS("LevelCancel"))  { levelCancel(); }
  else if (IS("Tare"))         { scaleJobStart(JOB_TARE, 0); }
  else if (IS("TareClear"))    { scaleTareClear(); }                   // testing: raw readings, no zero
  else if (IS("Balance"))      { scaleJobStart(JOB_BALANCE, 0); }
  else if (IS("BalanceCancel")) { scaleJobCancel(); }
  else if (IS("CalA"))         { scaleJobStart(JOB_CAL_A, v); }      // CalA<kg on arm A>
  else if (IS("CalB"))         { scaleJobStart(JOB_CAL_B, v); }
  else if (IS("CalC"))         { scaleJobStart(JOB_CAL_C, v); }
  else if (IS("Save"))         { settingsSave(); }
  else if (IS("Defaults"))     { settingsDefaults(); settingsSave(); laserApply(); }

  // ---- laser
  else if (IS("LaserOn"))        { laserOn = true;       laserApply(); }
  else if (IS("LaserOff"))       { laserOn = false;      laserApply(); }
  else if (IS("LaserToggle"))    { laserOn = !laserOn;   laserApply(); }
  else if (IS("ManualLaserOn"))  { manualLaser = true;   laserApply(); }
  else if (IS("ManualLaserOff")) { manualLaser = false;  laserApply(); }
  else if (IS("LaserAngle"))     { manualAngle = v;      laserApply(); }   // deg, used in manual mode
  else if (IS("LaserOffset"))    { cfg.laserOffset = v;  laserApply(); }

  // ---- switches
  else if (IS("AutoStopOn"))     { autoStop = true; }
  else if (IS("AutoStopOff"))    { autoStop = false; }
  else if (IS("MassCompOn"))     { massComp = true; }
  else if (IS("MassCompOff"))    { massComp = false; }
  else if (IS("TestMassOn"))     { testMassOn = true; }
  else if (IS("TestMassOff"))    { testMassOn = false; }
  else if (IS("LevelDebugOn"))   { dbgLevel = true; }
  else if (IS("LevelDebugOff"))  { dbgLevel = false; }
  else if (IS("ScaleDebugOn"))   { dbgScale = true; }
  else if (IS("ScaleDebugOff"))  { dbgScale = false; }
  else if (IS("LoopRateOn"))     { dbgLoopRate = true; }
  else if (IS("LoopRateOff"))    { dbgLoopRate = false; }

  // ---- everything below needs a number
  else if (!hasVal) return false;

  // ---- commissioning: run one actuator for 1 s at a signed PWM
  // (names have no digits: the parser reads digits as the value)
  else if (IS("JogLeft"))     { jogStart(0, v); }   // actuator 1, at 150 deg
  else if (IS("JogRight"))    { jogStart(1, v); }   // actuator 2, at 30 deg

  // ---- tire data (metric: mm and percent)
  else if (IS("RimD"))        { rimD = v * 0.001f; }
  else if (IS("TreadW"))      { treadW = v * 0.001f; }
  else if (IS("SideWall"))    { sideWallPct = v; }
  else if (IS("WeightR"))     { weightR = v * 0.001f; }

  // ---- leveling
  else if (IS("MinSpeed"))    { cfg.minSpeed = constrain(v, 0, 255); if (cfg.maxSpeed < cfg.minSpeed) cfg.maxSpeed = cfg.minSpeed; }
  else if (IS("MaxSpeed"))    { cfg.maxSpeed = constrain(v, cfg.minSpeed, 255); }
  else if (IS("Kp"))          { cfg.kp = max(v, 0.0f); }
  else if (IS("Ki"))          { cfg.ki = max(v, 0.0f); }
  else if (IS("Kd"))          { cfg.kd = max(v, 0.0f); }
  else if (IS("Alpha"))       { cfg.alpha = constrain(v, 0.0f, 0.99f); }
  else if (IS("LevelTol"))    { cfg.levelTol = constrain(v, 0.02f, 5.0f); }
  else if (IS("PitchOffset")) { cfg.pitchOffset = v; }
  else if (IS("RollOffset"))  { cfg.rollOffset = v; }
  else if (IS("PitchTrim"))   { pitchTrim = v; }
  else if (IS("RollTrim"))    { rollTrim = v; }

  // ---- scales
  else if (IS("TestMass"))    { cfg.testMass = v; }
  else return false;

  confirm(key, hasVal, v);
  return true;
}

// One token: letters, optionally followed by a number ("Kp35.5").
static void handleToken(char *tok) {
  char *p = tok;
  while (isalpha(*p)) p++;
  bool hasVal = (*p != 0);
  for (char *q = p; *q; q++) {
    if (!isdigit(*q) && *q != '.' && *q != '-') { reject(tok); return; }
  }
  if (p == tok) { reject(tok); return; }

  float v = hasVal ? atof(p) : 0;
  char key[16];
  uint8_t n = min((int)(p - tok), 15);
  memcpy(key, tok, n);
  key[n] = 0;
  if (!runCommand(key, hasVal, v)) reject(tok);
}

static void handleLine(char *line) {
  char *star = strrchr(line, '*');
  if (star) {
    uint8_t x = 0;
    for (char *q = line; q < star; q++) x ^= (uint8_t)*q;
    if (strlen(star + 1) != 2 || (uint8_t)strtol(star + 1, NULL, 16) != x) {
      reject("checksum");
      return;
    }
    *star = 0;
  }
  for (char *tok = strtok(line, " \t"); tok; tok = strtok(NULL, " \t")) handleToken(tok);
}

// Collect characters into lines without ever waiting for them.
void protocolPoll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0 && !lineOverflow && !lineNoise) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
      } else if (lineOverflow && !lineNoise) {
        reject("line too long");               // text only; noise is dropped silently
      }
      lineLen = 0;
      lineOverflow = false;
      lineNoise = false;
    } else if ((uint8_t)c < 0x20 && c != '\t' || (uint8_t)c > 0x7E) {
      // Not text: line noise, or the screen being programmed (its upload
      // traffic arrives here too). Drop the line without answering, so the
      // Nano never transmits into someone else's upload.
      lineNoise = true;
    } else if (lineLen < LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;     // too long: throw the whole line away
    }
  }
}
