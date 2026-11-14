// Settings.ino - tunable values and their EEPROM storage.
//
// "Save" stores the current values, "Defaults" goes back to the ones below.
// CalA / CalB / CalC save automatically.

void settingsDefaults() {
  cfg.magic       = SETTINGS_MAGIC;
  cfg.cal[0]      = 200000;     // placeholder - run CalA / CalB / CalC once
  cfg.cal[1]      = 200000;
  cfg.cal[2]      = 200000;
  cfg.pitchOffset = 1.52;       // set against a spirit level (2026-11-13)
  cfg.rollOffset  = -0.30;
  // Leveling, tuned on the machine with no tire (2026-11-13): 2-3 deg steps
  // finish in 1.7-1.9 s (occasionally ~2.5 s) and settle within 0.01-0.07 deg.
  cfg.minSpeed    = 20;         // both actuators still move at PWM 20 unloaded; 15 was no better
  cfg.maxSpeed    = 255;
  cfg.kp          = 200;
  cfg.ki          = 5;
  cfg.kd          = 10;         // brakes the approach; 20 was too much
  cfg.alpha       = 0.60;
  cfg.levelTol    = 0.10;
  cfg.laserOffset = 79.5;       // this servo's zero is ~80 deg off (82 pointed 2-3 deg CCW, 2026-11-13)
  cfg.testMass    = 5;
}

void settingsLoad() {
  EEPROM.get(0, cfg);
  if (cfg.magic != SETTINGS_MAGIC) settingsDefaults();
}

void settingsSave() {
  EEPROM.put(0, cfg);           // only rewrites bytes that changed
}

// Printed by the "Status" command (idle only - it is long).
void settingsPrint() {
  Serial.print(F("# V"));            Serial.println(F(FW_VERSION));
  Serial.print(F("# Cal "));         Serial.print(cfg.cal[0], 1);
  Serial.print(' ');                 Serial.print(cfg.cal[1], 1);
  Serial.print(' ');                 Serial.println(cfg.cal[2], 1);
  Serial.print(F("# PitchOffset ")); Serial.print(cfg.pitchOffset);
  Serial.print(F(" RollOffset "));   Serial.println(cfg.rollOffset);
  Serial.print(F("# MinSpeed "));    Serial.print(cfg.minSpeed, 0);
  Serial.print(F(" MaxSpeed "));     Serial.println(cfg.maxSpeed, 0);
  Serial.print(F("# Kp "));          Serial.print(cfg.kp);
  Serial.print(F(" Ki "));           Serial.print(cfg.ki);
  Serial.print(F(" Kd "));           Serial.println(cfg.kd);
  Serial.print(F("# Alpha "));       Serial.print(cfg.alpha);
  Serial.print(F(" LevelTol "));     Serial.println(cfg.levelTol);
  Serial.print(F("# LaserOffset ")); Serial.print(cfg.laserOffset, 1);
  Serial.print(F(" TestMass "));     Serial.println(cfg.testMass);
  Serial.print(F("# Tire mm RimD ")); Serial.print(rimD * 1000, 0);
  Serial.print(F(" TreadW "));       Serial.print(treadW * 1000, 0);
  Serial.print(F(" SideWall "));     Serial.print(sideWallPct, 0);
  Serial.print(F(" WeightR "));      Serial.println(weightR * 1000, 0);
  Serial.print(F("# AutoStop "));    Serial.print(autoStop);
  Serial.print(F(" MassComp "));     Serial.print(massComp);
  Serial.print(F(" TestMassOn "));   Serial.println(testMassOn);
}
