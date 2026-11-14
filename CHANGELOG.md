# Change Log

## Nano V0.6.0 + Screen V0.3.0 - machine testing (2026-11-12 to 11-13)

Both boards flashed and tested on the machine, each on its own USB port; the
link between them was tested through a PC relay, not yet wired directly.
Details and numbers are in GOALS.md.

Found on the hardware and fixed
- Both main loops read the clock before handling input, so "now - start"
  could wrap: Tare/Balance failed at once with `Err Scale`, Level would have
  stopped on its first tick, and the screen kept dropping the link.
- Lines longer than 48 characters were dropped silently; now `# ? line too long`.
- With both PID outputs at the limit the roll correction was lost; they are
  now scaled together.
- The frame MPU's roll had the wrong sign relative to the platform MPU (the
  platform MPU is mounted upside down).

New
- MPU orientation settings in machine axes (`TopAxisX/Y`, `BaseAxisX/Y`),
  so an MPU can be moved without code changes.
- Gyro back in as a complementary filter (`Gyro`), with the correct axes and
  a continuously tracked zero offset. Removes the filter lag.
- Reach area: `FindReach` drives to the four actuator end-stop combinations
  and stores how far the platform can tilt relative to the frame. `Level`
  refuses (`Err BaseTilt`) if level is not 1.5 deg inside it for the frame's
  tilt; replaces the single 13 deg limit. New errors `Reach`, `Axes`.
- Samples that are not tilt (jolts, I2C timeouts) are skipped; with the gyro
  on, the gyro carries the angle through them.
- Commissioning commands: `JogLeft`/`JogRight`/`JogTime`, `SensorDebugOn`,
  `TareClear`; scale debug shows readings per load cell.
- Both boards ignore non-text bytes, so they can stay wired together through
  a level converter while either one is programmed.
- Smaller number/checksum parsers instead of `atof`/`strtol` (saves ~1.4 KB flash).

Settings found on the machine (now the code defaults)
- Leveling: MinSpeed 20, MaxSpeed 255, Kp 300, Ki 5, Kd 10, Gyro 0.98,
  LevelTol 0.1 - 2-3 deg corrections in 1.6-1.8 s typically, no tire.
- PitchOffset 1.52, RollOffset -0.30 (spirit level), LaserOffset 79.5.
- Load cells with arms: A 100129, B 97834, C 102888 counts/kg (1.5 L bottle
  taken as 1.55 kg).
- Settings layout changed (EEPROM magic 0x7B08): flashing this firmware
  resets the Nano to these defaults.

Screen
- Speed sliders start at the tuned values; `Kp` slider 10-400.
- Error texts for `Reach` and `Axes`.

Saved
- The Nano's flash was read back before the first upload and turned out to
  be the lost V0.5 (docs/firmware-backup, with restore instructions).

## Nano V0.6.0 + Screen V0.3.0 (rebuild)

Nano V0.5 was lost with a corrupted flash drive. This is a rebuild from V0.4,
the V0.5 notes below and the screen code that was written against V0.5.

Repo
- One Nano sketch (`Nano/TireBalancer_Nano`) and one screen sketch (`Screen/ui`)
  instead of version-numbered files and zips. Old versions are in git history.
- Screen sketch, its bundled libraries and the SquareLine project are unpacked
  so changes show up in diffs.
- Added `docs/` (protocol, wiring, setup), `GOALS.md`, `.gitignore`, `Tools/build.ps1`.
- Removed the old test sketches; kept the MPU calibration, I2C scanner and
  touch test in `Tools/`.

Nano
- Rewritten as a non-blocking state machine (idle / leveling / measuring).
  No `delay()`, no `String`, no blocking serial reads. Sensors are only read
  while the job that needs them runs. RAM use 77% → 44%.
- Leveling loop runs at 50 Hz (was under 10 Hz because of `delay(100)`).
- PID per actuator replaces the cubic speed ramp. Output rides on `MinSpeed`
  because the actuators do not move below it; integral term for heavy loads.
- Actuator mixing now follows the platform geometry (pitch was under-driven 3x
  relative to roll).
- H-bridge protection: both inputs off for 150 ms before any start or reversal.
- Safety stops: frame tilted too far, platform sensor frozen or missing,
  leveling timeout, I2C timeout so a stuck bus cannot hang the loop.
- Angle from the accelerometer with the sensor's 5 Hz filter plus smoothing;
  the gyro is no longer used.
- Balance maths fixed: weight = contact radius x |sum of load vectors| / weight
  radius (V0.4 squared the imbalance). Readings are averaged before the angle
  is calculated, so results near 0°/360° no longer average wrongly.
- Tire data from the screen is used (V0.4 overwrote it with 215/65R15 every loop).
- New `WeightR`: radius where the weight is placed (clip-on vs stick-on).
- Load cell calibration built in (`CalA/B/C<kg>`), stored in EEPROM with the
  tuning values (`Save`, `Defaults`, `Status`).
- `Tare` averages 10 readings and is required once per power-up.
- Test mass is off by default (V0.4 always added 5 kg per cell).
- Mass compensation actually scales now (V0.4's integer `map()` made it 1 or 2).
- Servo pulse generated entirely by Timer2 interrupts, so its width no longer
  depends on the main loop.
- Serial: fixed-size line buffer, checksums, events repeated until `Ack`,
  `#` prefix for debug text. Command renames are listed in docs/Protocol.md.
- Dropped: `LDcycle`, `BDcycle`, `REon/REoff`.

Screen
- Matches the Nano's command set.
- Non-blocking serial receive (V0.2 could freeze the UI for 1 s per partial line).
- Link indicator in the corner with byte / bad-line counters; pop-up for Nano
  errors and for "no reply" timeouts.
- Units: Added Weight oz / g and Total Weight kg / lbs switches now work.
  Everything sent to the Nano is metric (rim diameter converted to mm).
- Tire Info reads the sliders when Enter is pressed (V0.2 sent 0 for a slider
  that had not been moved).
- New Clip-on / Stick-on switch on the Tire Info tab.
- Pitch / Roll sliders are a ±1.0° trim (tenths), as their labels say.
- "Speed Curve Exp." slider is now "Level Gain Kp" (5-100).
- Re-sends switch states and tire data whenever the Nano restarts.

# Older Nano history
Note there MAY be some information missing, see actual code comments for more info

## SD2_Nano_V0.1
- Combined Test Code:
  - Level_With_Serial
    - Changed the Actuation location logic from sin/cos axis length to vector addition
  - CM_Calculation
    - Angle calculation works but weightmass is a bit messed up still
  - Chatgpt6_WorseSpeedCtrl
    - Changed by giving each actuator proper proportioning for pitch and roll values
    - Angle acceleration compensation (alpha) actual works now
    - Magnitude of gravity calculation added for debug
- Added Serial Singular Commands:
  - start / stop (Begins / Stops Leveling)
  - LDon / LDoff (Shows / Hides Leveling debug)
  - BDon / BDoff (Shows / Hides Balancing debug)
  - calibrate (Sets current WA, WB, WC values to 0)
- Added Serial Variable Commands:
  - alpha"#" (sets alpha value)
  - minspeed"#" (sets min actuator speed value)
  - maxspeed"#" (sets max actuator speed value)
  - LDcycle"#" (shows Leveling debug for "#" of cycles)
  - BDcycle"#" (shows Balancing debug for "#" of cycles)

## SD2_Nano_V0.2
- Added Test Code:
  - ClocksPerCycle
  - Temp_SD_Laser (Added and Removed)
    - (Replaced With) No_Library_Servo_Laser
      - Basically this is code that inputs 0 to 360 degrees and turns a 180 degree servo and switches the laser to on the correct side
      - This recreates a pwm signal on a non-pwm pin and does not mess with the internal timers
      - This builds off No_Library_Servo
      - DO NOT USE Servo.h
        - The nano has 2 builtin timers (1&2) and for some reason Servo.h deactivates PWM on pins 9 and 10, aka actuator 1
- Added Mass Compensation
  - Takes the weight calculated on the load cells and adds voltage to increase the power to lift a tire while not over shooting velocity
- Added Serial Singular Commands:
  - Laseron / Laseroff (Enables / disables the laser and servo)
  - CPSon / CPSoff (Shows / Hides ClocksPerCycle)
  - MCon / MCoff (Enables / disables compensation for the actuators to get more voltage with more weight added)
- Added Serial Variable Commands:
  - RimD"#" (sets Rim Diameter value)
  - TreadW"#" (sets Tread Width value)
  - SideWallRatio"#" (sets the side wall percentage value)
  - TestMass"#" (Adds mass directly to WA,WB, and WC of the amount "#" in kg)
  - rollOffset"#" (Offsets the roll axis by "#" degrees)
  - pitchOffset"#" (Offsets the pitch axis by "#" degrees)
- Note:
  - Angle calculation works but weightmass is a bit messed up still
  - Since the servo code was rewritten, pulsewidth is not a linear corelation to degree angle
    - needs compensation still

## SD2_Nano_V0.3
- Added calibration for mpuBottom
- Added Base Angle calculation
  - under leveling code
  - error printed to serial if over MaxBaseAngle (13 degrees currently)
  - There is no reason to attempt to level if the actuators cannot reach their final destination
- Added BalanceState that controls when the average of WeightAngle and WeightMass is taken
  - Error message printed to serial and cancels Balancing if currently Leveling
- Added Serial Singular Commands:
  - Balance (Takes the average of WeightAngle and WeightMass)

## SD2_Nano_V0.4
- Added Laser Position Offset "LaserOffset" becasue the laser is about 90 degrees off from 0
  - Also fixed the accidental inversion of the laser angle
  - Added library Math.h to use "mod()"
- Also some serial bug fixes to get it to work with screen release "CYD_SD2_V0.1"
- Not going to lie this was worked on over a dual all nighter to get the screen code working before the final presentation
  - aka there is probably more that I added (Very Professional)
 
## SD2_Nano_V0.5
- Serial
  - Added "outputExponent" (Controls speed curve of the actuators) variable: "outputExponent"
  - Added TestMasson/off for debug
  - Added REon/REoff for ReportErrors on or off.
    - Currently just for the hx711 sensor errors, mainly for boot. OFF by default, turn ON for initial setup. Uses ReportErrorsState bool flag.
  - Commented out the delay(100); from loop
    - If REon is set then it will spam serial with error messages if the delay is not there
    - I think the hx711's are just slow? Anyway, the data read I've validated to still be accurate. So go figure if they say they aren't connected
    - Hopefully this might fix some esp and nano connection issues
  - Changed most rarly used serial writes to write from flash and not ram
    - SIGNIFICATLY reduces ram usage
    - Also fixed bugs where some serial writes would disappeear from hitting the ram cap
- Fixed TestMass Logic so it won't delete last sent TestMass amount using TestMassTemp
- Removed moment logic from ReadMass()
    - Apperently after testing and calibrating the load cells at a different location the the calibration value was very different from the previous calibration value. So after double checking, these load cells are not affected by the distance of force applied. They only count the mass applied in the vertical direction regardless of the force away from the sensor. I'd imagine that at some point this would become an issue at a further distance due to flex, but we are well within small angle approximation to not worry at 6 and under inches from the plate to the furthest measuring point.
- Added "Multi_HX711_Calibrate" to the test code
  - This gives you the values for Scale1Calibration, Scale2Calibration, and Scale3Calibration
- Fixed the Moment Balance Logic
- Fixed the RimD and TheadW so it actually updates with serial commands
- Removed XWeight,YWeight as they were bad duplicates of the centroid calculation
- Added outRangeWeight, inRangeWeight, %inRangeWeight, and inOverout
  - Basically this says how much weight is on the same side as the CMAngle
  - This is important because the center of mass calculation for how much weight to be needs to be added needs to be based on the mass difference of the side with the most weight and its opposite side
    - This cancels out the mass that would be balanced without the extra unbalancing weight, leaving only the mass that is needed to counter
  - ~This means that all past versions works for testing point masses but not practical tire masses~
  - ~TLDR old code not work, new code "should" be more acurate for all scenarios, needs testing but theory works~
  - Ok so after some testing with this new setup I've relized how I originally coded this (Sorry 4 month break). So these new values are cool to have but its just the same data we already had just in a different more understandable fasion. It does take more processing time to solve these new values but its really a debate on how accesable I want to make this for modifications. Likely, I will leave it in but comment it out when I get it working how I like.
  - By fixing the moment logic from ReadMass(), it also fixed a bug in how these values are calculated since that is the step before this calculation
  - inRangeWeight is basically CMweight
