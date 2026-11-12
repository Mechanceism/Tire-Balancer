// Settings.ino - tunable values and their EEPROM storage.
//
// "Save" stores the current values, "Defaults" goes back to the ones below.
// CalA / CalB / CalC save automatically.

void settingsDefaults() {
  cfg.magic       = SETTINGS_MAGIC;
  cfg.cal[0]      = 200000;     // placeholder - run CalA / CalB / CalC once
  cfg.cal[1]      = 200000;
  cfg.cal[2]      = 200000;
  cfg.pitchOffset = 2.0;        // measured with a level
  cfg.rollOffset  = 0.0;
  cfg.minSpeed    = 50;
  cfg.maxSpeed    = 100;
  cfg.kp          = 30;         // starting values - tune on the machine
  cfg.ki          = 20;
  cfg.kd          = 0;
  cfg.alpha       = 0.90;
  cfg.levelTol    = 0.20;
  cfg.laserOffset = 82;         // this servo's zero is 82 deg off
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
