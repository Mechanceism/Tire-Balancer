# Serial protocol (screen ⇄ Nano)

9600 baud, 8N1, plain text, one line per message. The same commands work typed
into Serial Monitor on the Nano's USB port (line ending: Newline).

The command names live in exactly two places, so a rename is a two-line change:

- Nano: `runCommand()` in [Protocol.ino](../Nano/TireBalancer_Nano/Protocol.ino)
- Screen: the `nanoSend(...)` calls in [ui.ino](../Screen/ui/ui.ino)

## Format

| Form | Meaning | Example |
|---|---|---|
| `Word` | an action or switch | `Level` |
| `Word<number>` | set a value | `Kp35.5` |
| several, space separated | handled left to right | `MinSpeed60 MaxSpeed120` |

Names are not case sensitive. Everything the Nano is given or reports is
**metric**: mm, kg, g, degrees. The screen does all unit conversion.

**Checksum.** A line may end in `*XX`, two hex digits holding the XOR of every
character before the `*` (the same idea as GPS/NMEA sentences). Examples:
`Level*56`, `Ping*30`, `LevelCompleted*1B`. The screen always adds it. If it is
present it must be correct or the line is dropped; hand-typed lines without it
are accepted by the Nano.

**Events and Ack.** `LevelCompleted`, `TareCompleted`, `BalanceCompleted ...`
and `Err ...` are sent with a checksum and repeated every 0.4 s (5 times at
most) until the screen answers `Ack`. In Serial Monitor you will simply see the
line a few times; type `Ack` to stop it.

**Lines starting with `#`** are information for a human (command echoes, debug
streams). The screen ignores them.

## Commands: screen → Nano

### Actions

| Command | What it does |
|---|---|
| `Level` | Start leveling. Ends with `LevelCompleted` (Auto Stop on) or runs until cancelled (Auto Stop off). |
| `LevelCancel` | Stop leveling. |
| `Tare` | Zero the load cells (arms empty). Needed once after every power-up. Ends with `TareCompleted`. |
| `TareClear` | Testing only: forget the zero point and use the raw readings as they are. |
| `Balance` | Weigh, calculate, switch the laser on. Ends with `BalanceCompleted ...`. |
| `BalanceCancel` | Abort a Tare / Balance / Cal that is still measuring. |
| `Stop` | Stop everything (actuators and measuring). |
| `Ping` | Nano answers `Pong`. The screen uses this for its link indicator. |
| `Ack` | "Got your event, stop repeating it." |
| `Status` | Print all settings (only while idle). |
| `Save` | Store the tuning values in EEPROM. |
| `Defaults` | Back to the built-in values (and save them). |

### Switches

| Command | Default | What it does |
|---|---|---|
| `AutoStopOn` / `AutoStopOff` | on | Off = keep hunting for level forever (demo mode). |
| `MassCompOn` / `MassCompOff` | off | More actuator drive as the load on the platform goes up (up to 2x at 20 kg). |
| `TestMassOn` / `TestMassOff` | off | Add `TestMass` kg to every load cell (bench testing). |
| `LaserOn` / `LaserOff` / `LaserToggle` | off | Laser + servo pointing at the last balance result. |
| `ManualLaserOn` / `ManualLaserOff` | off | Point at `LaserAngle` instead of the result. |
| `LevelDebugOn` / `LevelDebugOff` | off | `# P:.. R:.. B:.. A:..,..` 5x a second: pitch error, roll error, frame tilt, actuator PWM. |
| `ScaleDebugOn` / `ScaleDebugOff` | off | `# W:a,b,c T:total Ang:.. M:..` 2x a second: live load cells and result. |
| `LoopRateOn` / `LoopRateOff` | off | `# LoopRate n` once a second. |

### Values

| Command | Unit | Saved by `Save` | Notes |
|---|---|---|---|
| `RimD<n>` | mm | no | Rim diameter (the screen converts from inches). |
| `TreadW<n>` | mm | no | Tread / section width. |
| `SideWall<n>` | % | no | Aspect ratio. |
| `WeightR<n>` | mm | no | Radius where the balance weight is placed. 0 or never sent = rim radius. |
| `MinSpeed<n>` | PWM 0-255 | yes | Lowest PWM that still moves the actuators. |
| `MaxSpeed<n>` | PWM 0-255 | yes | |
| `Kp<n>` | PWM per deg | yes | PID proportional gain. |
| `Ki<n>` | PWM per deg·s | yes | PID integral gain. |
| `Kd<n>` | PWM per deg/s | yes | PID derivative gain (0 = PI controller). |
| `Alpha<n>` | 0-0.99 | yes | Angle smoothing. Higher = smoother but slower. |
| `LevelTol<n>` | deg | yes | Pitch and roll must both be inside this to count as level. |
| `PitchOffset<n>` / `RollOffset<n>` | deg | yes | Platform sensor reading when the platform is truly level. |
| `PitchTrim<n>` / `RollTrim<n>` | deg | no | Fine adjustment from the screen sliders, added to the offset. |
| `LaserOffset<n>` | deg | yes | Servo zero vs. platform zero. |
| `LaserAngle<n>` | deg | no | Target in manual laser mode. |
| `TestMass<n>` | kg | yes | Amount added per load cell while TestMass is on. |
| `CalA<n>` / `CalB<n>` / `CalC<n>` | kg | automatically | Calibrate one load cell with a known mass on that arm. See [Setup.md](Setup.md). |
| `JogLeft<n>` / `JogRight<n>` | signed PWM | no | Commissioning: run actuator 1 (left, 150°) or 2 (right, 30°) for 1 s, then stop. Positive = up. |

Command names never contain digits: the parser reads the first digit as the
start of the value (`Jog1100` would be `Jog` = 1100).

## Messages: Nano → screen

| Message | Meaning |
|---|---|
| `Ready V0.6.0` | Sent once at power-up. The screen re-sends its switch states and tire data. |
| `Pong` | Answer to `Ping`. |
| `LevelCompleted` | Platform is level and the actuators are off. |
| `TareCompleted` | Load cells zeroed. |
| `BalanceCompleted WeightAngle<deg> WeightMass<g> TotalWeight<kg>` | The result, 2 decimals each. `WeightMass` is in **grams**. |
| `Err <code>` | Something was refused or went wrong (table below). |
| `# ...` | Human-readable text, ignored by the screen. `# ? word` means "unknown command". |

### Error codes

| Code | Meaning |
|---|---|
| `Busy` | A different job is still running. |
| `BaseTilt` | The frame is tilted 13° or more; the actuators cannot reach level. |
| `MPUTop` | Platform angle sensor missing or frozen. |
| `MPUBottom` | Frame angle sensor missing. |
| `LevelTimeout` | Not level after 120 s (Auto Stop mode only). |
| `Scale` | A load cell gave no reading for 1.5 s. |
| `NoTare` | Balance or Cal before the first Tare since power-up. |
| `TireData` | Balance before the tire size was sent. |
| `NoLoad` | Less than 1 kg on the load cells. |
| `Cal` | Calibration mass missing, or the load cell reading barely changed. |

## Old → new names

Renames from the V0.4 / V0.5 commands. These are proposals: nothing depends on
the exact spelling, and changing one is a quick edit in the two places named at
the top of this page.

| Old | New | Why |
|---|---|---|
| `Start` | `Level` | Matches `Balance`; "start" did not say what starts. |
| `Stop` | `LevelCancel` (and `Stop` now stops everything) | Matches `BalanceCancel`. |
| `calibrate` | `Tare` | It zeroes the scales (the button already says Tare); real calibration is now `CalA/B/C`. |
| `MCon` / `MCoff` | `MassCompOn` / `MassCompOff` | Spelled out. |
| `LDon` / `LDoff` | `LevelDebugOn` / `LevelDebugOff` | Spelled out. |
| `BDon` / `BDoff` | `ScaleDebugOn` / `ScaleDebugOff` | It shows the load cells. |
| `CPSon` / `CPSoff` | `LoopRateOn` / `LoopRateOff` | Spelled out. |
| `MLA<n>` | `LaserAngle<n>` | Spelled out. |
| `SideWallRatio<n>` | `SideWall<n>` | Shorter; the screen already sent this. |
| `outputExp<n>` | `Kp<n>` (+ `Ki`, `Kd`) | The cubic speed curve was replaced by a PID loop. |
| `pitchOffset` from the slider | `PitchTrim<n>` | The slider is a ±1.0° fine trim on top of the saved `PitchOffset`. |
| `WeightAngleAvg`, `WeightMassAvg` | `WeightAngle`, `WeightMass` | Part of the `BalanceCompleted` line now. |
| `RimD<inches>` | `RimD<mm>` | Nano is metric only. |

Removed: `LDcycle` / `BDcycle` (print N debug lines) and `REon` / `REoff`
(the load cell errors they hid are now a single `Err Scale`).
