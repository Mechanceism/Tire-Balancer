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

Tuned values (no tire, 2026-11-13): `MinSpeed20 MaxSpeed255 Kp200 Ki5 Kd10
Alpha0.6 LevelTol0.1`. 2-3° corrections take 1.7-1.9 s and settle within
0.01-0.07°. About 0.35 s of that is mechanical slack before the platform starts
moving, plus 0.3 s of "is it still level" at the end.

Commissioning helpers: `JogLeft<pwm>` / `JogRight<pwm>` run one actuator for
1 s. With `LevelDebugOn` you can see which way, and how far, each one moves.

Keep each command line under 48 characters (`# ? line too long` otherwise).

## Other one-off calibrations

- **Laser zero**: `ManualLaserOn`, `LaserAngle0`, then adjust `LaserOffset<n>`
  until the dot sits at 0° (to the right). `Save`.
- **Frame MPU offsets**: run `Tools/MPU6050_Calibrate` and copy the numbers into
  `levelInit()` in `Level.ino`.
- **Balance weight radius**: `CLIP_ON_OFFSET_MM` and `STICK_ON_OFFSET_MM` at the
  top of `Screen/ui/ui.ino` are estimates (+10 mm and -15 mm from the rim
  radius). Measure where your weights actually sit.
