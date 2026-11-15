# Setup: upload, first run, calibration, tuning

## Uploading

With the level converter installed the boards can stay connected while
uploading. See "Leaving it connected permanently" in [Wiring.md](Wiring.md),
including when to switch the other board off.

### Nano

1. Install two libraries with the Library Manager: **MPU6050** by Electronic
   Cats and **HX711** by Rob Tillaart.
2. Open `Nano/TireBalancer_Nano/TireBalancer_Nano.ino` (the other tabs open with it).
3. Board: *Arduino Nano*. Processor: *ATmega328P (Old Bootloader)* if a plain
   *ATmega328P* upload fails.
4. Upload.

### Screen

The CYD is picky, so the exact libraries it needs are bundled in `Screen/libraries`.

1. Arduino IDE > File > Preferences > **Sketchbook location** = the `Screen`
   folder of this repo (the folder that contains `ui` and `libraries`).
   Set it back afterwards if you use the IDE for other projects.
2. Open `Screen/ui/ui.ino`.
3. Boards Manager: install **esp32** by Espressif Systems.
4. Board: *ESP32 Dev Module*. Tools > Flash Mode: *DIO*.
5. Upload. If the upload will not start, try a USB 2 port.

To edit the screens, open `Screen/squareline/CYD_SD2_V0.2.spj` in SquareLine
Studio and export into `Screen/libraries/ui`. A few widgets are added or
re-labelled in code (`setupExtraWidgets()` in `ui.ino`), so they will not show
up in SquareLine.

### Command line (optional)

`powershell -ExecutionPolicy Bypass -File Tools\build.ps1` compiles both
sketches with `arduino-cli` without uploading.

## First run after this rewrite

The leveling code is new and has only been compiled and simulated, not run on
the machine. Do the first tests with **no tire** and a hand on the power switch.

1. **Nano alone, on USB.** Open Serial Monitor at 9600, line ending *Newline*.
   You should see `Ready V0.6.0`. Type `Ping` → `Pong`. Type `Status`.
2. **Sensors.** `LevelDebugOn` prints pitch/roll error and frame tilt. Tilt the
   platform by hand and check the numbers follow. `LevelDebugOff`.
3. **Direction check.** With the platform clearly off level, type `Level` and
   watch the first second. It must move *toward* level. If it moves away, type
   `Stop` (or cut power) and note which way each actuator went: the fix is the
   sign of `MIX_ROLL` or of the errors in `levelTick()` (`Level.ino`).
4. **Screen.** Reconnect the serial wires. The corner should turn to `Nano OK`.
   If not, see [Wiring.md](Wiring.md).

## Load cell calibration (once)

The built-in factor (200000 counts/kg) is a placeholder, so results mean
nothing until this is done. You need one known mass, 2-10 kg.

1. Arms empty: `Tare`
2. Put the mass on arm **A** only (210°): `CalA5.0` (your mass in kg)
3. Move it to arm **B** (90°): `CalB5.0`
4. Move it to arm **C** (330°): `CalC5.0`

Each step prints `# CalA <factor>` and saves itself. A negative factor is fine;
it only means that load cell is wired the other way round.

Check with `ScaleDebugOn`: the mass on one arm should read its true weight on
that load cell and about zero on the other two.

## Settings kept by the screen

The screen keeps these in its own flash and sends them to the Nano after every
handshake (power-on, Nano restart), so they survive a power cycle: Pitch/Roll
Trim, Min/Max Speed and Level Gain Kp (as last sent with **Enter Speed**), Mass
Comp, Auto Stop, the units switches and Clip-on/Stick-on. Changes are saved
about 1.5 s after the last touch. Not kept on purpose: Manual Laser (so the
laser never switches itself on at boot) and Test Weight (a bench-test fake
load that would skew every result).

**Restore Factory Defaults** (bottom of the Debug tab) puts those back to the
tuned values - trims 0, Min 20, Max 150, Kp 300, Mass Comp on, Auto Stop on,
oz / kg, Clip-on - and sends them to the Nano. Calibration, the reach area, the
factory zero and the Recents list are not touched.

**Recents** (Tire Info tab, scroll down): the last 5 different sizes entered,
newest first. Tap one to fill in the format and size, then press **Enter**.

## Normal use

1. Arms empty → **Tare** (once per power-up).
2. Place the tire, centred on the jig.
3. **Level**.
4. **Tire Info**: set width, aspect ratio and rim diameter, choose Clip-on or
   Stick-on weight, press **Enter**.
5. **Balance**. Read the weight and angle; the laser points at the spot.

## Tuning the leveling

All of these can be typed while it is running. `Save` keeps them.

| Symptom | Try |
|---|---|
| Does not move, or hums without moving | Raise `MinSpeed`. |
| Slow to get there | Raise `Kp`, then `MaxSpeed`. |
| Overshoots and comes back, or hunts back and forth | Lower `Kp`; lower `MinSpeed` if it still moves; raise `LevelTol`. |
| Creeps the last bit very slowly under a heavy tire | Raise `Ki`, or switch Mass Comp on. |
| Twitches now and then while holding level | Raise `Alpha` (0.95) or `LevelTol`. |
| Stops slightly off level every time | Adjust `PitchOffset` / `RollOffset` (measure with a level), then `Save`. |

Tuned values (no tire): `MinSpeed20 MaxSpeed150 Ramp600 Kp300 Ki5 Kd10
Gyro0.98 LevelTol0.1`. With the 150 cap and the ramp (2026-11-14), 2-3°
corrections take 1.3-1.5 s and settle within 0.05° - faster than the earlier
uncapped 255 (1.6-1.8 s), because the gentler start avoids the jolt through
the mechanical slack. The cap is there for the 45 W supply; "Enter Speed" on
the screen now saves the speeds too. About 0.35 s of that is mechanical slack
before the platform starts moving, plus 0.3 s of "is it still level" at the end.

The gyro removes the lag of the angle filter, which is what allowed `Kp` 300
without overshoot. Its axes (pitch = +gyro Y, roll = -gyro X) were measured on
this machine; if the platform MPU is ever remounted, re-check them with
`SensorDebugOn` and a jog, or set `Gyro0` to fall back to the accelerometer.

## Reach area (frame tilt limit)

The platform can only tilt so far relative to the frame. `FindReach` drives the
actuators to all four end-stop combinations (about 45 s, keep clear) and saves
the result. `Level` then refuses with `Err BaseTilt` if level is not at least
1.5° inside that area for the frame's current tilt, and hunting (Auto Stop off)
stops if the frame is moved out of range. `M:` in the level debug line shows
the current margin.

How far the frame may tilt depends on which side is high (measured 2026-11-14,
after the frame MPU's roll sign was fixed):

| Frame side raised | Max tilt |
|---|---|
| 0° (right) | 23.5° |
| 45° | 12.1° |
| 90° (far side, actuators) | 11.5° |
| 135° | 10.3° |
| 180° (left) | 20.0° |
| 225° | 12.5° |
| 270° (screen side) | 15.4° |
| 315° | 14.7° |

(An earlier table showed only ~3.4° toward the far side; that came from the
frame's roll being read backwards.)

## MPU mounting (orientation settings)

Both MPUs are read in machine axes, the same ones used for the load cells and
laser: **+X toward 0° (right), +Y toward 90° (far side), +Z up**. For each MPU
you tell the Nano which sensor axis points along machine +X and which along
machine +Y. Use the axis arrows printed on the MPU board:

| Value | Meaning |
|---|---|
| `1` / `-1` | sensor X / opposite of sensor X |
| `2` / `-2` | sensor Y / opposite of sensor Y |
| `3` / `-3` | sensor Z / opposite of sensor Z (for a board mounted on its side) |

Commands: `TopAxisX<n> TopAxisY<n>` (platform MPU), `BaseAxisX<n> BaseAxisY<n>`
(frame MPU), then `Save`. Machine Z is worked out from X and Y, so an upside-down
board needs nothing extra. Defaults are in `Config.h`.

Current machine (2026-11-14): platform `TopAxisX2 TopAxisY1`, frame
`BaseAxisX2 BaseAxisY1`.

Checking a mounting with `SensorDebugOn` (`T:` = platform pitch,roll; `F:` =
frame pitch,roll). Send `Stop` first: with the actuators off the platform rides
on the frame, so **both MPUs must change by the same amount, the same way**.

1. Lift the frame at the **270° side** (screen side): pitch (first number of
   `T:` and `F:`) goes **down** on both; roll barely changes.
2. Lift the frame at the **0° side** (right): roll (second number) goes **up**
   on both; pitch barely changes. (The 180° side works too: roll goes down.)

Both lifts are needed - one checks pitch, the other roll. If one MPU moves the
wrong way, flip the sign of that MPU's setting for that axis (`BaseAxisY` for
pitch, `BaseAxisX` for roll, or the `TopAxis` ones). After the platform MPU is
changed, also confirm with a jog that the platform, not just the readings,
moves the right way.

Do not rely on the `Z:` field (whether each MPU's Z reads "up"). On this
machine it disagreed with the lift test: going by it, the frame's roll was set
backwards, which made the reach check refuse easy frame angles and attempt
impossible ones (fixed 2026-11-14).

After changing a mounting: re-check `PitchOffset`/`RollOffset` with a spirit
level, run `FindReach` again, and (platform MPU) confirm leveling does not
overshoot - with wrong gyro axes it would.

Commissioning helpers: `JogLeft<pwm>` / `JogRight<pwm>` run one actuator for
1 s. With `LevelDebugOn` you can see which way, and how far, each one moves.

Keep each command line under 48 characters (`# ? line too long` otherwise).

## Other one-off calibrations

- **Laser direction**: on the Debug tab switch **Manual Laser** on, step the
  angle with **-** / **+** until the dot points exactly at the 90° side (the
  far side, straight away from the screen), then press **Set 90**. The Nano
  shifts and saves its `LaserOffset` so that angle now counts as 90°
  (command: `LaserSet90`).
- **Frame MPU offsets**: run `Tools/MPU6050_Calibrate` and copy the numbers into
  `levelInit()` in `Level.ino`.
- **Balance weight radius**: `CLIP_ON_OFFSET_MM` and `STICK_ON_OFFSET_MM` at the
  top of `Screen/ui/ui.ino` are estimates (+10 mm and -15 mm from the rim
  radius). Measure where your weights actually sit.
