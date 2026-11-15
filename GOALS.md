# Goals

## What this machine is for

A portable, static tire balancer. A wheel is laid flat on a platform that
levels itself, three load cells find where the wheel's centre of gravity is,
and the machine tells the user how much weight to add and points a laser at
where to put it.

## Design goals

- **Level first.** The centre-of-gravity maths is only valid when the platform
  is level, whatever the ground is doing. Target: within 0.1° (achieved, with
  and without a tire).
- **Simple to use.** Tare (empty), place the tire, Level, enter the tire size,
  Balance.
- **Honest numbers.** Results to 2 decimals for now; the final rounding will
  match the weight sizes you can actually buy, decided from test data.
- **Safe.** Slow, static test. The actuator H-bridge can never have both
  directions on. A sensor fault stops the motion instead of chasing it, and
  the platform is never asked to reach a position it cannot get to.
- **Easy to modify.** Text commands you can type by hand, tunable values
  changeable without re-flashing, code split by job.

## Division of work

- **Nano**: everything real-time (leveling, weighing, laser). Metric only.
- **Screen**: the user interface, all unit conversion, and the user settings
  (kept in its flash and sent to the Nano after every handshake).

## Next up

1. **Balance a real wheel and check the result**: balance, add the weight it
   asks for where the laser points, balance again - the remaining imbalance
   should be close to zero. Then the repeatability test below.
2. Measure where clip-on and stick-on weights really sit and set
   `CLIP_ON_OFFSET_MM` / `STICK_ON_OFFSET_MM` (Screen/ui/ui.ino; now estimates).
3. Re-check the load cells with a weighed reference mass (calibrated with a
   1.5 L bottle taken as 1.55 kg).
4. With a heavier wheel (car wheel, 15-25 kg), tune the loaded gains there
   too: the interpolation has one loaded point (8.66 kg) and holds its line
   only up to 2x that.
5. Free Nano flash before the next Nano feature (98% used) - see below.

## Status

### Done and tested on the machine

Firmware and link (2026-11-12):
- [x] Nano firmware rebuilt as a non-blocking state machine
- [x] Screen and Nano use the same command set (docs/Protocol.md), with
      checksums, Ack/resend and a link indicator
- [x] Old firmware read back first (docs/firmware-backup), including the lost V0.5

Motion and leveling (2026-11-13/14):
- [x] Both actuators move the right way; roll/pitch ratio matches the mixing
      maths. The right actuator needed a driver DIP switch changed.
- [x] Offsets set against a spirit level (PitchOffset 1.52, RollOffset -0.30);
      re-checked: platform reads 0 deg on both axes after leveling
- [x] Gyro back in (complementary filter 0.98). V0.4 had pitch/roll on the
      wrong gyro axes and no zero correction.
- [x] Tuned: MinSpeed 20, MaxSpeed 150 (45 W supply), Ramp 600, Kp 300, Ki 5,
      Kd 10, Gyro 0.98, LevelTol 0.1. 2-3 deg corrections in 1.3-1.5 s.
- [x] Hunting after a cold power-on fixed: the power-on gyro zero was ~10 deg/s
      out. Now learned only while the platform is still; confirmed by a cold
      power cycle.
- [x] Samples that are not tilt (jolts, bad I2C reads) are skipped
- [x] MPU orientation is a setting in machine axes (TopAxisX/Y, BaseAxisX/Y).
      Checked by lifting the frame with the actuators off: 270 deg side for
      pitch, 0/180 deg side for roll (the frame's roll had been backwards).
- [x] Reach area (FindReach, also a button on the Debug tab): allowed frame
      tilt 10.3-23.5 deg depending on direction (table in docs/Setup.md),
      confirmed on the machine. Old code used a single 13 deg.

Load compensation (2026-11-14, 4.80-12 trailer tire, 8.66 kg):
- [x] Under 8.66 kg the actuators still move at PWM 15; no-load gains level it
      in 1.2-1.6 s. Load-tuned set: Kp 300, Ki 5, Kd 15, MinSpeed 20.
- [x] Mass Comp: Level weighs first (~0.3 s) against the saved factory zero
      (SetZero, empty platform) and interpolates the gains linearly (up to 2x
      8.66 kg). Works from power-on with or without a tire; read 8.49-8.65 kg
      over 13 runs.

Balance and laser (2026-11-13/14):
- [x] Load cells calibrated with the arms fitted: A 100129, B 97834,
      C 102888 counts/kg (1.5 L bottle taken as 1.55 kg)
- [x] Direction check passed: bottle on one arm gives the opposite angle and
      the laser points there
- [x] Laser calibration on the screen (-/+ and Set 90)

Screen (2026-11-14):
- [x] Link fixed: IO22 (TX) / IO35 (RX) through the level converter (the old
      RXD pin is held by the board's USB-serial chip)
- [x] Tire size by format dropdown + full-screen number pad; Recents
- [x] Settings kept across power cycles; Restore Factory Defaults
- [x] No restart when Level is refused (two screen animations at once)

### Still to do on the machine

- [ ] Balance a wheel, add the weight, re-balance (see Next up)
- [ ] Repeatability test: same wheel, 10 runs; then with known added weights
- [ ] Decide result rounding from that data
- [ ] Measure real weight positions (clip-on / stick-on offsets)
- [ ] Re-check the load cells with a weighed reference mass
- [ ] Small cross-talk seen: bottle on A showed ~22 g on C, on B ~7 g on C
- [ ] Check the laser angle across the range (servo is not linear)
- [ ] Tune the loaded gains with a heavier wheel

## Ideas for later

- **Nano flash is at 98%** (about 390 bytes free). Before adding more to the
  Nano: shorten the Status printout, drop the TestMass feature, or move to the
  newer Optiboot bootloader (+1.5 KB; needs a spare Arduino or USBasp once,
  erases the Nano's saved settings, which the code defaults already match).
- Servo linearity table for the laser
- Move the code-built widgets (Tire Info, laser row, Debug buttons) into the
  SquareLine project
