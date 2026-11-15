# Setup: upload, first run, calibration, tuning

## Uploading

The boards can stay wired together. The screen uploads normally; for the Nano,
switch the screen off (or hold it in reset) during the upload - see "Uploading
with everything connected" in [Wiring.md](Wiring.md).

The Nano's flash is 98% full (about 390 bytes free) - see GOALS.md before
adding to it.

Each board's libraries are in the repo, at the exact versions tested, so
nothing has to be installed from the Library Manager:

| Board | Sketchbook location | Libraries bundled |
|---|---|---|
| Nano | `Nano` | `Nano/libraries`: MPU6050 (Electronic Cats) 1.4.3, HX711 (Rob Tillaart) 0.6.0 |
| Screen | `Screen` | `Screen/libraries`: LVGL 8.3.11 (with `lv_conf.h`), TFT_eSPI 2.5.43 (set up for this board), `ui` (the screens) |

The Arduino IDE only looks for libraries in the `libraries` folder of its
**Sketchbook location**, so point that at the board's folder before opening
its sketch:

1. Arduino IDE > File > Preferences > **Sketchbook location** > Browse, pick
   the board's folder from the table (the folder that contains `libraries`),
   OK.
2. **Restart the Arduino IDE** (it reads the libraries at start-up).
3. Open the sketch, set the board (below) and Upload.

Switching between the Nano and the screen means changing the Sketchbook
location and restarting again. Write down your old location first and set it
back afterwards if you use the IDE for other projects. Do not install or update
these libraries with the Library Manager while the location points into the
repo: it would write into the repo's copy.

The board packages (Boards Manager) are not in the repo. Tested with
**Arduino AVR Boards 1.8.6** and **esp32 by Espressif Systems 3.3.0**.

### Nano

1. Sketchbook location = the repo's `Nano` folder, restart the IDE.
2. Open `Nano/TireBalancer_Nano/TireBalancer_Nano.ino` (the other tabs open with it).
3. Board: *Arduino Nano*. Processor: *ATmega328P (Old Bootloader)* if a plain
   *ATmega328P* upload fails (it does on this machine's Nano).
4. Upload. With the screen wired up, see "Uploading with everything connected"
   in [Wiring.md](Wiring.md).

`Tools/MPU6050_Calibrate` and `Tools/I2C_Scanner` (Nano) build with the same
setting.

### Screen

The CYD is picky about library versions and settings, which is why its
libraries are bundled.

1. Sketchbook location = the repo's `Screen` folder, restart the IDE.
2. Open `Screen/ui/ui.ino` (`Web.ino` and `WebPage.h` open with it).
3. Boards Manager: install **esp32** by Espressif Systems.
4. Board: *ESP32 Dev Module*. Tools > Flash Mode: *DIO*.
5. Tools > Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**. With the
   web page the program is about 1.3 MB, more than the default scheme's
   1.25 MB, and the IDE stops with "Sketch too big" if this is left on Default.
   (`partitions.csv` in the sketch folder makes sure the board gets this
   layout either way. Saved settings and Recents are kept when switching.)
6. Upload. If the upload will not start, try a USB 2 port.

To edit the screens, open `Screen/squareline/CYD_SD2_V0.2.spj` in SquareLine
Studio and export into `Screen/libraries/ui`. Several parts are built or
re-arranged in code in `setupExtraWidgets()` (`ui.ino`) and will not show up
in SquareLine: the whole Tire Info tab (format, number pad, Clip/Stick-on,
Recents), the laser row and Set 90 at the top of the Debug tab, Find Reach,
Restore Factory Defaults, and the status line.

### Command line (optional)

`powershell -ExecutionPolicy Bypass -File Tools\build.ps1` compiles both
sketches with `arduino-cli` without uploading. It uses only the repo's
libraries (the same redirect as above), so it also proves nothing is missing.

## After rebuilding or changing the hardware

The machine has been through all of this (2026-11-13/14) and the results are
the firmware defaults. Repeat a step only if that part changes. First tests
with **no tire** and a hand on the power switch.

1. **Nano alone, on USB.** Open Serial Monitor at 9600, line ending *Newline*.
   You should see `Ready V0.7.0`. Type `Ping` → `Pong`. Type `Status`.
2. **MPU mounting:** see "MPU mounting" below (lift the frame at 270° and 0°).
3. **Direction check.** `JogLeft100` then `JogLeft-100`, same for `JogRight`,
   with `LevelDebugOn`: raising an actuator must move its corner up. Then
   with the platform clearly off level, `Level` must move *toward* level.
   If not, `Stop` (or cut power).
4. **Factory zero:** with the platform empty (arms on), `SetZero`.
5. **Reach area:** Find Reach on the Debug tab (or `FindReach`), tire off.
6. **Screen:** the corner should read `Nano OK`. If not, see [Wiring.md](Wiring.md).

## Load cell calibration (once)

Done 2026-11-13 with the arms fitted (factors are the firmware defaults).
Repeat with a weighed mass of 2-10 kg if the load cells or arms change.

**From the screen:** Debug tab > **Load Cell Calibration** (above Find Reach)
opens "Select which load cell to calibrate": **A (210)**, **B (90)**,
**C (330)**. The first pick asks for the arms to be empty and zeroes the scale.
Then the number pad asks for the weight on that load cell (kg, or lbs if the
total weight is shown in lbs); put the weight on that arm only and press
Enter. "Calibrating Load Cell A..." takes about a second, then you are back at
the selection with a tick on the ones done: pick the next one or skip it.
**Done** (Cancel before any is done) lists the new factors and how much each
changed. Cancel on the number pad or the loading page goes back to the
selection.

**From the web page:** the same button on the Debug tab opens a screen with
Tare and a row per load cell (weight box + Calibrate), then Done.

Each load cell is saved by the Nano as soon as it is calibrated; ending early
leaves the ones not done as they were. Any of them can be done on its own.

**By hand** (Serial Monitor, or the web console):

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
Trim, Min/Max Speed (as last sent with **Enter Speed**), Mass Comp, Auto Stop,
the units switches and Clip-on/Stick-on. Changes are saved about 1.5 s after
the last touch. Not kept on purpose: Manual Laser (so the laser never switches
itself on at boot) and Test Weight (a bench-test fake load that would skew
every result).

The leveling gains (`Kp`, `Ki`, `Kd` and the loaded set) are not on the
screens: they are tuned once and kept by the Nano. Change them from a PC or the
web page's console if the machine is ever re-tuned (see "Tuning the leveling").

**Restore Factory Defaults** (bottom of the Debug tab) puts those back to the
tuned values - trims 0, Min 20, Max 150, Mass Comp on, Auto Stop on, oz / kg,
Clip-on - and sends them to the Nano. Calibration, the reach area, the factory
zero and the Recents list are not touched.

**Recents** (Tire Info tab, scroll down): the last 5 different sizes entered,
newest first. Tap one to fill in the format and size, then press **Enter**.

## Normal use

1. Arms empty → **Tare** (once per power-up; Balance needs it).
2. Place the tire, centred on the jig.
3. **Level**. It weighs the platform first and picks its gains for that load,
   so it works with or without a tire and without a Tare.
4. **Tire Info**: choose the format, tap the size box and type the size as
   printed (or tap a Recent), choose Clip-on or Stick-on, press **Enter**.
5. **Balance**. Read the weight and angle; the laser points at the spot. It
   switches itself off after 30 s. Pressing **Laser** (touch screen or web)
   cancels that: the laser then stays on or off as you leave it.

## Web page (phone or laptop)

The screen makes its own WiFi hotspot, so a phone can run the machine:

1. On the phone, join the WiFi **TireBalancer** (open, no password). It has
   no internet; tell the phone to stay connected if it asks.
2. The page usually opens by itself as a "sign in to network" page. If not,
   open a browser at **http://192.168.4.1**. (The touch screen's Debug tab,
   at the bottom, shows this too, with the number of phones connected.)

The page has the same tabs as the touch screen (Level, Tire Info, Balance,
Debug) plus:

- **STOP** at the top, always available: stops everything on the Nano.
- **Safety tick** - "I can see the machine and it is clear". Until it is
  ticked, everything that moves the platform or switches the laser on
  (Level, Balance, Laser, Find Reach, manual laser, console commands that are
  not read-only) is greyed out, and the screen refuses those requests anyway.
  It clears itself after 5 minutes unused or when the WiFi link drops.
- **Tire Info**: type the size with the phone's keyboard (format is found
  automatically); it shows what the size means as you type.
- **Level tab, Live view** (choose *Leveling*, *Sensors* or *Off*; it starts
  Off each time the page opens):
  - *Leveling*: a bubble level of the platform's distance from level (the dot
    sits on the high side; rings at 0.1, 1 and 2 deg), frame tilt, reach margin
    with "Level can start / Close to the limit / Too steep", both actuators'
    drive, and two 20 s charts (platform off level, with the level tolerance
    band; actuator drive). Level starts and finishes are marked on the charts,
    and the time of each run is listed under the Level button. Pause, Save CSV
    (last 2 minutes) and tap/drag on a chart to read values.
  - *Sensors*: bubble levels for the platform (accelerometer only) and the
    frame, and the **reach map**: how far, and which way, the frame may tilt
    for the platform to still reach level (outer line), the line Level must
    start inside (1.5 deg margin), and the frame's position now.
- **Balance tab, wheel drawing**: the wheel seen from above, standing at the
  screen, to scale once the size is entered: the three arms, where the weight
  goes (with grams/oz) and the centre of gravity (enlarged). With **Live load
  cells** on, the arms show their kg and the drawing follows the load live.
- **Balance tab, Balance report** (after a Balance): date and time (from the
  phone), weight and angle, total, centre of gravity, the tire, weight type
  and radius, each arm's load and how level the platform was right after the
  Balance, Mass Comp / Test Weight, firmware. **Save CSV**, or **Print / Save
  PDF** (one page with the wheel drawing). Nothing is kept on the machine; save
  the report if you need it. Saving and printing need the phone's own browser
  (http://192.168.4.1), not the WiFi sign-in window.
- **Console**: every line sent to and received from the Nano, a box to type
  commands, **Save** to download the log (with times) as a text file, and
  the full command list from [Protocol.md](Protocol.md) with a short
  description each: tap to send, or to put a value command in the box.
- **Info**: link counters, firmware versions, uptime, free memory, and the
  Nano's settings from its last `Status`, one per row.

The touch screen and the page always agree: a change on either shows on the
other (and on every connected phone) within a second, and anything started
from a phone shows the same Leveling/Balancing page on the touch screen, with
its Cancel button working as usual.

Live data comes from the Nano's own debug outputs (`LevelDebug`,
`SensorDebug`, `ScaleDebug`). The screen switches each one on only while some
phone is showing it and off again about 3 s after the last one stops (closing
the page or switching tabs is enough), so the 9600-baud link never carries
streams nobody is looking at. Typing those commands in the console still works,
but a live view may switch them back off when it closes. The reach map uses
the Nano's `Status`, which the screen reads when the link comes up and again
after Find Reach.

The page is in `Screen/ui/WebPage.h` (plain HTML/CSS/JavaScript, edit it
directly); the server side is `Screen/ui/Web.ino`.

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
