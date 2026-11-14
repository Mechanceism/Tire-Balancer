/*
  Self Leveling Static Tire Balancer - Arduino Nano firmware

  The Nano does the real-time work: it levels the platform, weighs the tire on
  three load cells, works out the balance weight and points the laser at the
  spot where it goes. The touch screen (ESP32 "CYD") is only the user interface
  and talks to this sketch with short text commands - see docs/Protocol.md.

  Tabs:
    TireBalancer_Nano.ino  shared state, setup(), loop(), debug streams
    Level.ino              MPU6050s, PID leveling, actuator drive (H-bridge safety)
    Balance.ino            HX711 load cells, tare / calibrate / balance maths
    Laser.ino              servo pulse on Timer2 + laser selection
    Protocol.ino           serial commands in, events out
    Settings.ino           defaults and EEPROM storage

  Nothing in loop() is allowed to block: no delay(), no String, no waiting on
  serial. Sensors are only read while the job that needs them is running.

  Libraries: MPU6050 (Electronic Cats) and HX711 (Rob Tillaart).
  Do NOT use Servo.h - it takes Timer1 and kills PWM on pins 9/10 (actuator 1).
*/
#include <Wire.h>
#include <EEPROM.h>
#include <MPU6050.h>
#include <HX711.h>
#include "Config.h"

// ------------------------------------------------------------ shared state
uint8_t  state = ST_IDLE;
Settings cfg;

// Options the screen switches on and off (not saved)
bool  autoStop   = true;    // stop leveling once level (off = keep hunting forever)
bool  massComp   = false;   // more actuator drive when the platform is loaded
bool  testMassOn = false;   // add cfg.testMass to every load cell (bench testing)
float pitchTrim  = 0;       // deg - fine adjust from the screen, added to cfg.pitchOffset
float rollTrim   = 0;

// Tire data from the screen, in metres (0 = not entered yet)
float rimD        = 0;      // rim diameter
float treadW      = 0;      // tread (section) width
float sideWallPct = 0;      // aspect ratio in percent
float weightR     = 0;      // radius where the balance weight is placed (0 = rimD / 2)

// Live readings
float weight[3]  = {0, 0, 0};   // kg on load cells A, B, C
uint8_t hxReads[3] = {0, 0, 0}; // readings per load cell since the last scale debug line
uint16_t rejectedSamples = 0;   // platform MPU samples skipped as not-tilt: jolts or bad reads (since power-up)
float pitchF     = 0;           // filtered platform pitch, deg
float rollF      = 0;           // filtered platform roll, deg
float baseAngle  = 0;           // frame tilt, deg (total)
float basePitch  = 0;           // frame pitch / roll, deg
float baseRoll   = 0;
float reachMargin = 99;         // deg that level is inside the platform's reach on this frame
float pitchRaw   = 0;           // platform angle from this tick's accelerometer reading, deg
float rollRaw    = 0;
int16_t gyroX    = 0;           // platform gyro, raw counts (131 per deg/s)
int16_t gyroY    = 0;

// Last balance result
float resAngle   = 0;       // deg - where the weight goes
float resMassG   = 0;       // g   - how much weight
float resTotalKg = 0;       // kg  - tire + wheel

// Laser
bool  laserOn     = false;
bool  manualLaser = false;  // point at manualAngle instead of the balance result
float manualAngle = 0;

// Debug streams
bool dbgLevel    = false;
bool dbgSensor   = false;
bool dbgScale    = false;
bool dbgLoopRate = false;

// ------------------------------------------------------------ setup / loop
void setup() {
  actuatorsInit();                 // H-bridge inputs low before anything else
  Serial.begin(SERIAL_BAUD);
  settingsLoad();
  laserInit();
  scaleInit();
  levelInit();
  Serial.print(F("Ready V"));
  Serial.println(F(FW_VERSION));
}

void loop() {
  static uint32_t lastLevelTick = 0;

  // Commands first, THEN read the clock: anything a command starts gets a
  // timestamp <= now, so "now - start" can never wrap round to a huge number.
  protocolPoll();
  uint32_t now = millis();

  if (now - lastLevelTick >= LEVEL_PERIOD_MS) {
    float dt = (now - lastLevelTick) * 0.001f;
    lastLevelTick = now;
    if (state == ST_LEVELING || state == ST_REACH || dbgLevel || dbgSensor) levelTick(now, constrain(dt, 0.01f, 0.1f));
  }
  jogTick(now);

  if (state == ST_SAMPLING || dbgScale || (state == ST_LEVELING && massComp)) scalePoll(now);

  eventTick(now);
  debugTick(now);
}

// Stop everything that moves or measures. Safe to call at any time.
void stopAll() {
  actuatorsStop();
  scaleJobCancel();
  state = ST_IDLE;
}

// ------------------------------------------------------------ debug streams
// Every debug line starts with '#', which the screen ignores. Lines are only
// printed when they fit in the transmit buffer so they can never stall loop().
void debugTick(uint32_t now) {
  static uint32_t lastLevel = 0, lastScale = 0, lastRate = 0, lastSensor = 0;

  // Raw sensors: platform accel angles, frame angles, platform gyro (counts).
  // Kept under 50 characters so it never waits for the transmit buffer.
  if (dbgSensor && now - lastSensor >= 100 && Serial.availableForWrite() > 50) {
    lastSensor = now;
    Serial.print(F("# T:"));  Serial.print(pitchRaw);
    Serial.print(',');        Serial.print(rollRaw);
    Serial.print(F(" F:"));   Serial.print(basePitch, 1);
    Serial.print(',');        Serial.print(baseRoll, 1);
    Serial.print(F(" G:"));   Serial.print(gyroX);
    Serial.print(',');        Serial.println(gyroY);
  }
  static uint32_t loops = 0;
  loops++;

  if (dbgLoopRate && now - lastRate >= 1000) {
    lastRate = now;
    if (Serial.availableForWrite() > 24) {
      Serial.print(F("# LoopRate "));
      Serial.println(loops);
    }
    loops = 0;
  }

  if (dbgLevel && now - lastLevel >= 100 && Serial.availableForWrite() > 50) {
    lastLevel = now;
    Serial.print(F("# P:"));  Serial.print(pitchF - cfg.pitchOffset - pitchTrim);
    Serial.print(F(" R:"));   Serial.print(rollF - cfg.rollOffset - rollTrim);
    Serial.print(F(" B:"));   Serial.print(baseAngle, 1);
    Serial.print(F(" A:"));   Serial.print(actuatorOutput(0));
    Serial.print(',');        Serial.print(actuatorOutput(1));
    Serial.print(F(" X:"));   Serial.print(rejectedSamples);       // samples skipped (jolts / bad reads)
    Serial.print(F(" M:"));   Serial.println(reachMargin, 1);      // deg level is inside the reach area
  }

  if (dbgScale && now - lastScale >= 500 && Serial.availableForWrite() > 60) {
    lastScale = now;
    float angle, massG, total;
    bool ok = balanceCompute(weight, angle, massG, total);
    Serial.print(F("# W:"));  Serial.print(weight[0], 3);
    Serial.print(',');        Serial.print(weight[1], 3);
    Serial.print(',');        Serial.print(weight[2], 3);
    Serial.print(F(" T:"));   Serial.print(total);
    if (ok) {
      Serial.print(F(" Ang:")); Serial.print(angle, 1);
      Serial.print(F(" M:"));   Serial.print(massG, 1);
    }
    // readings per load cell in this 0.5 s: about 5 each at 10 SPS, 40 at 80 SPS
    Serial.print(F(" n:"));   Serial.print(hxReads[0]);
    Serial.print(',');        Serial.print(hxReads[1]);
    Serial.print(',');        Serial.println(hxReads[2]);
    hxReads[0] = hxReads[1] = hxReads[2] = 0;
  }
}
