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
- [ ] **Right actuator (2, D5/D6): does not move at any PWM (100-255), either
      direction** - hardware between the pins and the motor
- [x] Balance -> screen -> Ack -> laser, angle and mass verified by hand
- [ ] Look at where the laser pointed for 36.7 deg and the 0/90/180/270 sweep

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

Ideas for later:

- Servo linearity table for the laser
- Work out actuator length from the two MPUs to warn before end of stroke
- Move the weight-position switch and the Kp slider into the SquareLine project
