# Goals

## What this machine is for

A portable, static tire balancer. A wheel is laid flat on a platform that
levels itself, three load cells find where the wheel's centre of gravity is,
and the machine tells the user how much weight to add and points a laser at
where to put it.

## Design goals

- **Level first.** The centre-of-gravity maths is only valid when the platform
  is level, whatever the ground is doing. Target: within 0.3° (to be confirmed
  by measurement).
- **Simple to use.** Tare, place the tire, Level, enter the tire size, Balance.
- **Honest numbers.** Results to 2 decimals for now; the final rounding will
  match the weight sizes you can actually buy, decided from test data.
- **Safe.** Slow, static test. The actuator H-bridge can never have both
  directions on. A sensor fault stops the motion instead of chasing it.
- **Easy to modify.** Text commands you can type by hand, tunable values
  changeable without re-flashing, code split by job.

## Division of work

- **Nano**: everything real-time (leveling, weighing, laser). Metric only.
- **Screen**: the user interface and all unit conversion.

## Status

Done in code, compiles, **not yet run on the machine**:

- [x] Nano firmware rebuilt as a non-blocking state machine (V0.6.0)
- [x] PID leveling with minimum-speed handling and direction-change dead time
- [x] Balance maths corrected (moment balance about the contact radius)
- [x] Load cell calibration and settings stored in EEPROM
- [x] Screen and Nano use the same command set (docs/Protocol.md)
- [x] Unit selection on the screen (oz / g, kg / lbs)
- [x] Clip-on vs stick-on weight position
- [x] Link indicator, checksums and resend so a lost message is visible

Bench-tested on the real boards (2026-11-12), each board on its own USB:

- [x] Nano: boots, Ping, Status, both MPUs read, all three HX711s at 10 SPS,
      Tare, Balance with test mass, error events, Ack, checksum rejection
- [x] Screen: pings, syncs settings on link-up and after `Ready`, Acks events,
      ignores corrupted lines (tested by playing the Nano from the PC)
- [x] Old firmware read back first (docs/firmware-backup) - includes V0.5

Motion and end-to-end tests (2026-11-13), boards linked through a PC relay:

- [x] Left actuator (1, D9/D10): moves both ways at PWM 100; direction and
      roll/pitch ratio match the mixing maths
- [x] Right actuator (2, D5/D6): did not move until a driver DIP switch was
      changed; now matches the left one (about 1.1 deg pitch per second at PWM 100)
- [x] Balance -> screen -> Ack -> laser, angle and mass verified by hand
- [x] Laser sweep read 2-3 deg CCW; LaserOffset 82 -> 79.5 (saved)
- [x] First full level: 5.1 deg -> under 0.2 deg in 4 s, LevelCompleted at
      8.9 s, 0.14-0.17 deg after stopping (default gains, no tire)

To do on the machine:

- [ ] Look at the screen: results panel, units switches, corner status, pop-ups
- [ ] Install the level converter and connect the boards

- [ ] Confirm the actuators move toward level (direction check in docs/Setup.md)
- [ ] Confirm the screen hears the Nano; if not, the RX-pin test in docs/Wiring.md
- [ ] Calibrate the three load cells (CalA / CalB / CalC)
- [ ] Tune Kp / Ki / MinSpeed / LevelTol and measure the achieved level
- [ ] Check the laser angle against a protractor (servo is not linear)
- [ ] Measure real weight positions and set the clip-on / stick-on offsets
- [ ] Repeatability test: same wheel, 10 runs; then with known added weights
- [ ] Decide result rounding from that data

Leveling tuning (2026-11-13, no tire):

- [x] Offsets set against a spirit level: PitchOffset 1.52, RollOffset -0.30
- [x] Actuators move at PWM 20 unloaded -> MinSpeed 20 (was 50)
- [x] MaxSpeed 255, Kp 200, Ki 5, Kd 10, Alpha 0.6, LevelTol 0.1:
      2-3 deg steps done in 1.7-1.9 s (was 2.7-6.4 s), settle within 0.07 deg
- [x] Samples that are not tilt (jolts, bad I2C reads) are skipped
- [ ] **Load boost (to implement):** with a tire on, the actuators need more
      drive. The old code added a boost from the measured tire weight. Mass
      Comp is a first version (scales drive up to 2x at 20 kg) but must be
      checked once the load cells are calibrated: MinSpeed probably has to
      rise with load too (PWM 20 moves an empty platform, maybe not a loaded
      one), and the boost should help the final slow approach, not just top
      speed. Then re-tune with a tire on.

- [x] Reach area measured (FindReach): level must be inside it for the
      frame's tilt. Allowed frame tilt depends on direction (about 7 deg in
      some directions, over 20 in others), unlike the old single 13 deg.
- [ ] Check the frame MPU is mounted the same way round as the platform's
      (lift one side of the frame, see docs/Setup.md)
- [x] Gyro back in (complementary filter 0.98). V0.4 had pitch/roll on the
      wrong gyro axes and no zero correction (gyro X read -4 deg/s at rest).
      With it, Kp 300: 1.6-1.8 s typical, overshoot ~0.02 deg.

Ideas for later:

- Flash is at 87%: keep an eye on it (the load boost must fit)

- Servo linearity table for the laser
- Work out actuator length from the two MPUs to warn before end of stroke
- Move the weight-position switch and the Kp slider into the SquareLine project
