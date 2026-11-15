# Wiring

## Arduino Nano

| Nano pin | Goes to | Notes |
|---|---|---|
| D0 (RX) | Screen IO22 (via level converter) | Shared with the Nano's USB. |
| D1 (TX) | Screen IO35 (via level converter) | Shared with the Nano's USB. |
| D2 / D3 | HX711 **A** DOUT / SCK | Load cell at 210° |
| D7 / D8 | HX711 **B** DOUT / SCK | Load cell at 90° |
| D11 / D12 | HX711 **C** DOUT / SCK | Load cell at 330° |
| D4 | Laser servo signal | Pulse made by Timer2. Do not use `Servo.h`. |
| D9 / D10 | Actuator 1 UP / DOWN (PWM) | Actuator at 150° |
| D5 / D6 | Actuator 2 UP / DOWN (PWM) | Actuator at 30° |
| A2 / A3 | Laser 1 / Laser 2 | Laser 1 covers 0-180°, laser 2 covers 180-360° |
| A4 / A5 | I2C SDA / SCL | Platform MPU6050 at 0x68, frame MPU6050 at 0x69 (AD0 high) |
| GND | Screen GND | The two boards must share ground. |

Free pins: D13, A0, A1 (A6 and A7 are analog input only).

**H-bridge safety.** UP and DOWN of the same actuator must never be on
together. Only `actuatorDrive()` in `Level.ino` writes those pins; it switches
both off and waits 150 ms (`DEAD_TIME_MS`) before any start or reversal.

## Platform layout (top view)

```
                 90°  load cell B
                  |
   actuator 1     |     actuator 2
      150°  \     |     /  30°
             \    |    /
   180° ------ centre (laser servo) ------ 0°   (+X)
             /    |    \
      210°  /     |     \  330°
  load cell A     |     load cell C
                  |
                270°  fixed pivot, screen faces the user here
```

Angles are counter-clockwise from the right, like a normal graph.

## Power

- One 12 V supply (45 W) feeds the actuator H-bridge drivers and a dedicated
  12 V to 5 V converter. The converter feeds the Nano's 5 V pin **and** the
  screen's 4-pin connector (5 V, GND). The screen is not powered through the
  Nano's own regulator, which could not handle its current.
- 45 W is not enough for both actuators at full speed: `MaxSpeed` is capped at
  150 and `Ramp` softens the start-up current.
- Power-up order: the 12 V supply first.

## Screen (JC2432W328C) through a logic level converter

Use a bidirectional level converter (the common BSS138 MOSFET type) between
the boards. The Nano's 5 V TX straight into the ESP32's 3.3 V RX pin is over
its rated voltage.

| Screen (low side) | Converter | Nano (high side) |
|---|---|---|
| 3.3 V | LV | |
| | HV | 5 V |
| GND | GND | GND |
| **IO22** (TX) | LV1 ⇄ HV1 | D0 (RX) |
| **IO35** (RX) | LV2 ⇄ HV2 | D1 (TX) |

The link uses **IO22 and IO35**, not TXD/RXD on the 4-pin serial connector.
Those are also wired to the board's own USB-serial chip, which held RXD high
so the Nano's replies never arrived: the corner showed `No link rx:0 bad:0`
even with the level converter fitted (2026-11-14). The 4-pin connector is
still used for **5 V and GND** to power the screen.

The converter's **LV pin must be on 3.3 V**. Left unconnected, the pull-up
resistors on its LV side tie the channels together, so the screen hears its own
transmissions. The corner then shows **"RX hears own TX"**.

With the screen's USB connected, its port (115200 baud) prints a live log:
`> ` lines sent to the Nano, `< ` lines received, `!` link events.

- **LV must be the ESP32's 3.3 V.** The screen board's pin header has a 3.3 V
  pin. The VCC on the 4-pin serial plug is 5 V (power input).
- Ground must be shared.

### Uploading with everything connected

- **Uploading to the screen (USB):** works as normal. The Nano only transmits
  when spoken to and ignores the screen's upload traffic.
- **Uploading to the Nano (USB):** the screen's TX, driven through the
  converter, holds the Nano's RX line, so the PC cannot reach the Nano while
  the screen is running. Either switch the screen off for the upload, or hold
  it in reset through its own USB port (RTS asserted, DTR not - this is what
  `holdesp` did during development), upload, then release it.
- **Commanding the Nano from a PC** with everything wired: type into the
  screen's USB port (115200 baud, newline). The screen passes each line on to
  the Nano and its log shows the replies (`< ` lines). Opening the screen's
  port restarts the screen, not the Nano.
- **The computer:** the converter does not connect either board's power to the
  other's USB. Return the actuator current to its supply on its own heavy
  wires, not through the logic ground or a USB cable.

## If the screen cannot hear the Nano

The bottom-left corner of the screen shows the link state:

| Corner text | Meaning |
|---|---|
| `Nano OK` (green) | The screen is hearing valid lines from the Nano. |
| `No link rx:0 bad:0` | Not a single byte is arriving on IO35. |
| `No link rx:<counting> bad:<counting>` | Bytes arrive but are garbled. |
| `RX hears own TX` | The converter's LV pin is not on 3.3 V. |
| `Paused: noise on RX` | Several garbled bytes in a row (e.g. during a Nano upload); sending pauses 10 s. |

History: from V0.4 until 2026-11-14 the screen never heard the Nano because
its RX was on the 4-pin plug's RXD (GPIO 3), which the board's USB-serial chip
holds high. Moving RX to IO35 (and TX to IO22) fixed it.
