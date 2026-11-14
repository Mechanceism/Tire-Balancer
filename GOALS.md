# Goals

## What this machine is for

A portable, static tire balancer. A wheel is laid flat on a platform that
levels itself, three load cells find where the wheel's centre of gravity is,
and the machine tells the user how much weight to add and points a laser at
where to put it.

## Design goals

- **Level first.** The centre-of-gravity maths is only valid when the platform
  is level, whatever the ground is doing. Target: within 0.1° (achieved with no
  tire; confirm with a tire on).
- **Simple to use.** Tare, place the tire, Level, enter the tire size, Balance.
- **Honest numbers.** Results to 2 decimals for now; the final rounding will
  match the weight sizes you can actually buy, decided from test data.
- **Safe.** Slow, static test. The actuator H-bridge can never have both
  directions on. A sensor fault stops the motion instead of chasing it, and
  the platform is never asked to reach a position it cannot get to.
- **Easy to modify.** Text commands you can type by hand, tunable values
  changeable without re-flashing, code split by job.

## Division of work

- **Nano**: everything real-time (leveling, weighing, laser). Metric only.
- **Screen**: the user interface and all unit conversion.

## Next up

1. **Connect the boards** through the level converter (docs/Wiring.md) and
   test the real link. If the screen still cannot hear the Nano, use the
   corner counters and the RX-pin test in docs/Wiring.md. So far the two
   boards have only talked through a PC relay.
2. **Balance direction check** with the bottle on one arm: bottle on arm A
   (210 deg) should give a weight angle of about 30 deg, laser pointing there.
3. **Load boost (to implement).** With a tire on, the actuators need more
   drive. The old code added a boost from the measured tire weight. Mass Comp
   is a first version (scales drive up to 2x at 20 kg) and must now be tested
   with a real tire: MinSpeed probably has to rise with load too (PWM 20 moves
   an empty platform, maybe not a loaded one), and the boost should help the
   final slow approach, not just top speed. Then re-tune with a tire on.
4. Re-check the load cells with a weighed reference mass.

## Status

### Done and tested on the machine

Firmware and link (2026-11-12):
- [x] Nano firmware rebuilt as a non-blocking state machine (V0.6.0)
- [x] Screen and Nano use the same command set (docs/Protocol.md), with
      checksums, Ack/resend and a link indicator
- [x] Nano bench test: Ping, Status, both MPUs, all three HX711s at 10 SPS,
      Tare, Balance, error events, Ack, checksum rejection
- [x] Screen bench test (PC playing the Nano): pings, re-syncs after `Ready`,
      Acks events, ignores corrupted lines. Display, units switches and
      corner status checked by eye.
- [x] Old firmware read back first (docs/firmware-backup), including the lost V0.5

Motion and leveling (2026-11-13):
- [x] Both actuators move the right way; roll/pitch ratio matches the mixing
      maths. The right actuator needed a driver DIP switch changed.
- [x] Offsets set against a spirit level (PitchOffset 1.52, RollOffset -0.30);
      re-checked: platform reads 0 deg on both axes after leveling
- [x] Gyro back in (complementary filter 0.98). V0.4 had pitch/roll on the
      wrong gyro axes and no zero correction (one axis read -4 deg/s at rest).
- [x] Tuned, no tire: MinSpeed 20, MaxSpeed 255, Kp 300, Ki 5, Kd 10,
      Gyro 0.98, LevelTol 0.1. 2-3 deg corrections in 1.6-1.8 s typically
      (was 2.7-6.4 s), settling within ~0.03-0.09 deg.
- [x] Samples that are not tilt (jolts, bad I2C reads) are skipped
- [x] Reach area measured (FindReach): level must be inside it for the
      frame's tilt. Allowed frame tilt depends on direction: only ~3.4 deg
      with the far (90 deg) side high, up to ~21 deg with the 225 deg side
      high (table in docs/Setup.md). Old code used a single 13 deg.
- [x] MPU orientation is a setting in machine axes (TopAxisX/Y, BaseAxisX/Y);
      platform MPU is upside down, frame MPU face up (checked with a block
      under the 270 deg side, docs/Setup.md)

Balance and laser (2026-11-13):
- [x] Balance -> screen -> Ack -> laser through the relay; angle and mass
      checked by hand
- [x] Laser sweep read 2-3 deg CCW; LaserOffset 82 -> 79.5 (saved)
- [x] Load cells calibrated with the arms fitted, using a 1.5 L bottle taken
      as 1.55 kg: A 100129, B 97834, C 102888 counts/kg, each reads 1.55 back.
      Fitting the arms changed the factors by 1.6-8% vs bare cells.

### Still to do on the machine

- [ ] Connect the boards through the level converter (see Next up)
- [ ] Balance direction check with a known load on one arm (see Next up)
- [ ] Load boost and tuning with a tire on (see Next up)
- [ ] Re-check the load cells with a weighed reference mass
- [ ] Small cross-talk seen: bottle on A showed ~22 g on C, on B ~7 g on C
- [ ] Check the laser angle against a protractor (servo is not linear)
- [ ] Measure real weight positions and set the clip-on / stick-on offsets
      (`CLIP_ON_OFFSET_MM`, `STICK_ON_OFFSET_MM` in Screen/ui/ui.ino)
- [ ] Repeatability test: same wheel, 10 runs; then with known added weights
- [ ] Decide result rounding from that data
- [ ] Optional (mechanical): level sits near the actuators' retracted ends.
      Moving the mounts so level is nearer mid-stroke would allow more slope
      toward the far side.

## Ideas for later

- Flash is at 91%: keep an eye on it (the load boost must fit). Candidates
  to trim: the long Status printout, debug streams, or moving to the newer
  Optiboot bootloader (+1.5 KB, needs an ISP programmer once)
- Servo linearity table for the laser
- Move the weight-position switch and the Kp slider into the SquareLine project
